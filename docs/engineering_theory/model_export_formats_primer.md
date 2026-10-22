# Model Export Formats Primer (PyTorch, ONNX and NCNN)

## Purpose of This Document

This guide explains what it means to "export" or "convert" a trained neural network to a different model format, such as ONNX or NCNN, and why this can make inference much faster on a small computer like the Raspberry Pi 5 in the rover.

It starts with the basics: what a model file contains, what a CPU and a GPU are, and what a pickle is. It then explains why running a PyTorch `.pt` model on the Pi is slow, what an export does to the model, and how the result relates to the pan oscillation work in [pan_oscillation_vision_latency.md](../bug_fixing/pan_oscillation_vision_latency.md) (§5, Step 1).

Read it from top to bottom once. After that, the glossary at the end works as a quick reference.

---

## Part 1: Base Concepts — What a Model Actually Is

### 1.1 What Is a Neural Network Model?

A neural network is a large mathematical function. It takes an input (for us, an image) and produces an output (for us, a list of bounding boxes with class labels and confidence scores).

The function is built from many **layers** applied one after another. Each layer does a fairly simple operation, for example:

- **convolution:** slide small filters over the image to detect patterns such as edges, textures and shapes,
- **activation:** apply a simple non-linear function to every value (for example SiLU or ReLU),
- **normalisation:** rescale values so they stay in a well-behaved range,
- **upsampling, concatenation and pooling:** resize the data and combine it.

Stack enough of these layers and the network can recognise people.

### 1.2 Weights and Architecture

A model has two parts, which are easiest to understand separately:

- **Architecture:** the *recipe*. It says which layers exist, in what order, and how they connect.
- **Weights (parameters):** the *numbers* inside the layers, for example the values in every convolution filter. Training learns them.

An analogy: the architecture is the circuit diagram, and the weights are the component values.

For scale, YOLO11n (the "nano" model the rover uses) has about **2.6 million weights**. One 640×640 image takes roughly **6.5 billion floating-point operations** (6.5 GFLOPs) to process.

### 1.3 What Is a Tensor?

A tensor is a multi-dimensional array of numbers, the general form of a vector (1-D) and a matrix (2-D).

- A colour image is a 3-D tensor: `height × width × 3 channels`.
- A batch of images is 4-D: `batch × channels × height × width`.
- The weights of a convolution layer are a 4-D tensor too.

Everything flowing through a neural network is a tensor, so "running a model" means doing arithmetic on tensors.

### 1.4 Training vs Inference

A model's life has two distinct phases:

| | Training | Inference |
|---|---|---|
| Goal | learn the weights | use the weights to make predictions |
| Happens | once, on a powerful machine with a GPU | every frame, on the rover |
| Needs gradients? | yes (to know how to adjust each weight) | no |
| Weights change? | yes, constantly | no, they are fixed |
| What matters | flexibility, debuggability | speed, low latency, low memory |

This is the central idea of this whole document: **a tool built to be good at training is not necessarily good at inference.** The rover only ever does inference.

---

## Part 2: Hardware Concepts — Where the Maths Runs

### 2.1 What Is a CPU?

The **Central Processing Unit** is the general-purpose processor of a computer. It runs the operating system, Python, the pipeline and everything else.

Key properties:

- **A few powerful cores.** The Pi 5 has 4 cores (ARM Cortex-A76 at 2.4 GHz). Each core can run its own thread of instructions.
- **Built for complex, branching logic.** It is good at "if this, do that" code, and at jumping around in memory.
- **Caches.** These are small, very fast memories close to each core. Reading from main RAM is much slower than reading from cache, so code that reuses data while it is still in cache runs much faster.

### 2.2 What Is SIMD (and NEON)?

**SIMD** means *Single Instruction, Multiple Data*. It is a CPU feature that applies one instruction to several numbers at once.

Without SIMD, adding two arrays of 4 numbers takes 4 add instructions. With a 128-bit SIMD register holding four 32-bit floats, it takes **one** instruction.

- On ARM CPUs, the SIMD instruction set is called **NEON**.
- On Intel/AMD (x86) CPUs, it is called **SSE / AVX**.

Neural networks are mostly long runs of multiply-and-add over big arrays, which is exactly the work SIMD is built for. A library that uses NEON well can be several times faster than one that doesn't, on the same CPU.

### 2.3 ARM vs x86

An **instruction set architecture (ISA)** is the "language" of machine instructions a CPU understands. The two you'll meet most often are:

| | x86 (Intel / AMD) | ARM |
|---|---|---|
| Typical devices | desktops, laptops, servers | phones, tablets, Raspberry Pi, Apple Silicon |
| Design philosophy | complex instructions, high power | simpler instructions, power-efficient |
| SIMD extension | SSE, AVX, AVX-512 | NEON (and SVE on newer chips) |

The two ISAs are not compatible: code compiled for x86 can't run on ARM. More importantly for us, **a math library tuned for x86 isn't automatically tuned for ARM.** Hand-written AVX code has to be rewritten as NEON code, and a lot of software has had far more optimisation effort put into its x86 and GPU paths than into its ARM path.

### 2.4 What Is a GPU?

A **Graphics Processing Unit** was originally built to draw 3-D graphics, which means doing the same small calculation for millions of pixels at once. That turned out to be exactly what neural networks need too.

| | CPU | GPU |
|---|---|---|
| Number of cores | few (4–16), each powerful | thousands, each simple |
| Good at | branching logic, serial tasks | identical maths on huge arrays |
| Neural network speed | slow-ish | very fast |

PyTorch's fast path for neural networks is on **NVIDIA GPUs via CUDA** (NVIDIA's programming platform for its GPUs). Most training happens there.

### 2.5 Why the Pi Can't Just Use Its GPU

The Pi 5 does have a GPU (the VideoCore VII), but it is a small graphics chip, not a compute accelerator. PyTorch has **no CUDA support** for it, because CUDA is NVIDIA-only. As far as PyTorch is concerned, the Pi has no GPU at all.

On the rover, inference therefore runs **entirely on the four ARM CPU cores.** That makes it very important to use CPU code that is efficient on ARM.

### 2.6 Memory Bandwidth

**Memory bandwidth** is how fast data can move between RAM and the CPU. On small boards it is often the real bottleneck: the CPU can calculate faster than RAM can feed it numbers.

This is why "doing fewer passes over memory" (Part 5, operator fusion) can matter as much as "doing less arithmetic".

---

## Part 3: Software Concepts — What a `.pt` File Is

### 3.1 What Is a Deep Learning Framework?

A **framework** is a software library for building, training and running neural networks. It provides tensors, layers, automatic gradient calculation, GPU support and model saving and loading.

**PyTorch** (by Meta) is the most popular framework for research and training. **Ultralytics** (which provides YOLO) is built on top of PyTorch.

### 3.2 What Is Serialisation?

**Serialisation** means converting an in-memory object (with all its nested parts) into a stream of bytes, so it can be saved to a file or sent over a network. **Deserialisation** turns the bytes back into the object.

Examples you've already met in this project:

- JSON is a serialisation format (the JSONL telemetry logs, the `{"T":1001,...}` servo messages).
- YAML is too (the config files).

### 3.3 What Is a Pickle?

**Pickle** is Python's built-in serialisation format (the `pickle` module). It can save almost any Python object: lists, dictionaries, custom class instances and whole object trees.

How it works, roughly:

- When saving, pickle records **the name of each object's class** (for example `ultralytics.nn.tasks.DetectionModel`) plus the object's internal data.
- When loading, pickle **imports that class by name** and rebuilds the object.

Consequences:

1. **A pickle depends on the code that created it.** To load a pickled YOLO model you need the `ultralytics` and `torch` Python packages installed, in compatible versions. Without those exact class definitions, the file is unreadable.
2. **A pickle only works in Python.** A C++ program, a phone app or a microcontroller can't use it.
3. **Unpickling can run arbitrary code.** A malicious pickle can execute anything when loaded, so only load `.pt` files from sources you trust (such as the official Ultralytics releases). Recent PyTorch versions have a safer `weights_only=True` loading mode for this reason.

### 3.4 What Is Inside `yolo11n.pt`?

A `.pt` file is a **zip archive** containing:

- a **pickle** describing the Python model object: which classes, which layers and how they're configured,
- the raw **weight tensors** stored alongside it,
- training metadata (for example the class names and the training arguments).

The important point is that the `.pt` file stores **a Python object**, not a description of the computation. It says "here is an instance of `DetectionModel`, rebuild it". It doesn't say "here is the list of mathematical operations to perform". The operations exist only in the Python code of the `forward()` method of each layer class.

### 3.5 What Does "General-Purpose Engine" Mean?

When you run a `.pt` model, PyTorch is the "engine" doing the work. It is **general-purpose** because it is designed to support *everything* a researcher might want:

- any architecture, including ones that change shape every call,
- training *and* inference,
- automatic gradients (**autograd**) for every operation,
- CPUs, NVIDIA GPUs, Apple GPUs and more,
- debugging with normal Python tools: print statements and breakpoints inside the model.

That flexibility is great for training and research. But for one fixed model doing inference on one CPU, most of it is overhead you pay for and never use.

### 3.6 Eager Execution

By default, PyTorch runs in **eager mode**: each operation executes *immediately* when Python reaches that line of code.

```python
x = conv1(x)   # Python calls PyTorch -> runs a convolution -> returns a new tensor
x = bn1(x)     # Python calls PyTorch -> runs a batch-norm -> returns a new tensor
x = silu(x)    # Python calls PyTorch -> runs an activation -> returns a new tensor
...            # repeated for hundreds of layers, every frame
```

PyTorch never sees the "whole picture" of the model. It only sees one operation at a time, as Python asks for it. This is what makes PyTorch easy to debug, and it is also why PyTorch can't optimise across operations.

---

## Part 4: Why the PyTorch Model Is Slow on the Pi

Putting Parts 2 and 3 together, these are the main reasons:

### 4.1 Per-Operation Overhead (Python and Dispatch)

Every layer call goes through Python, then PyTorch's **dispatcher**. The dispatcher is the machinery that decides, at runtime, which version of the operation to run (CPU or GPU? float or int? autograd or not?). This costs a few microseconds per operation.

YOLO11n has hundreds of operations per frame. On a fast desktop the overhead is small next to the maths. On a Pi core, which is several times slower per operation, it adds up to a noticeable share of the frame time.

### 4.2 No Cross-Operation Optimisation

In eager mode, each operation:

1. reads its input tensor from memory,
2. computes,
3. writes a brand-new output tensor back to memory.

For a `conv → batch-norm → activation` sequence, that is **three full round trips** through memory for what is mathematically one combined step. As Part 2.6 explained, memory traffic is often the real bottleneck on a Pi.

### 4.3 Kernels Not Tuned for ARM

A **kernel** is the low-level routine that actually performs one operation, such as "3×3 convolution on float32 data". PyTorch's CPU kernels get most of their tuning effort on x86 (AVX). Its ARM/NEON paths work, but they are generally less optimised than those of a library written specifically for ARM.

### 4.4 Training Baggage

Even when you aren't training, PyTorch carries the infrastructure for it: autograd bookkeeping, a general-purpose memory allocator and support for dynamic shapes. Most of this can be switched off (`torch.no_grad()`, which Ultralytics already does for you), but the engine is still not *designed* around a single fixed inference job.

### 4.5 Summary

| Source of slowness | What causes it | What fixes it |
|---|---|---|
| Per-op overhead | Python + dispatcher on every layer | run the whole graph in compiled C++ |
| Memory round trips | each op writes a full tensor | operator fusion |
| Generic kernels | x86/GPU-first tuning | an ARM-specialised runtime (NCNN) |
| Training baggage | general-purpose design | an inference-only runtime |

Every row in the "fixes" column is what an export plus a dedicated runtime gives you.

---

## Part 5: What Exporting Actually Does

### 5.1 From a Python Object to a Computation Graph

A **computation graph** is a description of the model as pure maths. It is a network of nodes (operations such as `Conv`, `Add`, `Sigmoid`) connected by edges (the tensors flowing between them), with the weights stored as constants.

Think of the difference like this:

- `.pt` (eager PyTorch): *"Here is a Python program. Run it line by line to find out what it does."*
- Exported graph: *"Here is the complete wiring diagram of the calculation. Nothing else is needed."*

The graph doesn't need Python, PyTorch or Ultralytics to run. Any program that understands the format can execute it.

### 5.2 How the Graph Is Captured (Tracing)

The usual way to export is **tracing**:

1. Create a dummy input of the right shape (for example a 1×3×640×640 tensor of zeros).
2. Run it through the model once while PyTorch *records* every operation that happens.
3. Save that recorded sequence of operations, with the weights, as a graph.

Consequence: **the graph only contains what happened on that one run.** If the model had `if` statements that depend on the input, only the branch taken is recorded. YOLO's detection network doesn't change its operations from frame to frame, so this isn't a problem here.

### 5.3 Static Shapes (Fixed Input Size)

The exported graph is typically built for **one fixed input shape**, for example `1 × 3 × 640 × 640`. This is what the `imgsz` argument at export time controls.

Fixing the shape is a *feature*: if the runtime knows every tensor size in advance, it can:

- allocate all memory once at start-up instead of every frame,
- pick the best kernel for each exact layer size,
- lay out memory to suit the CPU cache.

The cost is that if you want a different input size later, for example 320×320, you have to re-export.

### 5.4 Graph Optimisations

Once the whole graph is visible, the exporter or runtime can rewrite it to be cheaper while computing exactly the same result.

**Constant folding.** Any part of the calculation that only involves constants (weights) is computed once at export time instead of every frame.

**Operator fusion.** Several consecutive operations are merged into one kernel, so the data is read from memory once, all the steps are done while it is in cache, and the result is written once.

The classic example is **folding batch-norm into the convolution**. A convolution followed by batch-norm computes:

$$
y = \gamma \cdot \frac{(W * x + b) - \mu}{\sqrt{\sigma^2 + \epsilon}} + \beta
$$

where:

- **$x$** is the input tensor and **$y$** the output.
- **$W$** and **$b$** are the convolution's weights and bias.
- **$*$** denotes convolution.
- **$\mu$** and **$\sigma^2$** are the mean and variance batch-norm learned during training.
- **$\gamma$** and **$\beta$** are batch-norm's learned scale and shift.
- **$\epsilon$** is a tiny constant that prevents division by zero.

At inference time, all of $\gamma, \beta, \mu, \sigma^2$ are fixed numbers. So we can pre-compute new convolution weights:

$$
W' = \frac{\gamma}{\sqrt{\sigma^2 + \epsilon}} \, W
\qquad
b' = \frac{\gamma (b - \mu)}{\sqrt{\sigma^2 + \epsilon}} + \beta
$$

and the two layers become a single convolution:

$$
y = W' * x + b'
$$

The batch-norm layer has disappeared completely, with identical output. Fusing the activation (for example SiLU) into the same kernel removes another memory round trip.

**Dead-code elimination.** Anything only needed for training (loss calculations, auxiliary outputs) is removed.

**Memory planning.** With all tensor sizes known, the runtime reuses the same memory buffers for tensors whose lifetimes don't overlap, which reduces the total memory used and cache misses.

### 5.5 Running the Graph: the Inference Runtime

The exported file is just a description. You still need a program that executes it, called an **inference runtime** or **inference engine**. It is typically written in C++ and does only one job: load a graph, then run it as fast as possible on a specific kind of hardware.

This split is the key concept:

| | Format (the file) | Runtime (the engine) |
|---|---|---|
| PyTorch | `.pt` (pickle) | PyTorch |
| ONNX | `.onnx` | ONNX Runtime (`onnxruntime`) |
| NCNN | `.param` + `.bin` | NCNN library |

---

## Part 6: The Model Formats

### 6.1 PyTorch (`.pt`)

- **What it is:** a pickled Python model object (Part 3.4).
- **Runtime:** PyTorch, in eager mode.
- **Strengths:** the native training format; the most flexible; easy to debug.
- **Weaknesses:** needs Python + PyTorch + Ultralytics; slow on small CPUs; large dependency footprint.
- **Use it for:** training, development and as the "master copy" you export from.

### 6.2 ONNX (`.onnx`)

**ONNX** stands for *Open Neural Network Exchange*. It is an open, vendor-neutral standard for describing computation graphs, backed by Microsoft, Meta and others.

- **What it is:** a single file containing the graph (nodes from a standard list of operators such as `Conv`, `Relu` and `Resize`) plus the weights.
- **Runtime:** usually **ONNX Runtime**, Microsoft's inference engine. It has pluggable "execution providers" for different hardware (CPU, CUDA, TensorRT, OpenVINO and more).
- **Strengths:** portable. The same file runs on Windows, Linux, ARM, x86 and GPUs, and from Python, C++, C# and other languages. Many other formats are built *from* ONNX, so it is often the middle step in a conversion chain.
- **Weaknesses:** on a Pi CPU, ONNX Runtime is usually faster than PyTorch but slower than a runtime written specifically for ARM.
- **Analogy:** ONNX is to neural networks what PDF is to documents: a widely understood exchange format.

### 6.3 NCNN (`.param` + `.bin`)

**NCNN** is an open-source inference framework from Tencent, written specifically for **mobile and embedded ARM CPUs** (it was built to run models inside phone apps).

- **What it is:** two files, usually in one directory:
  - `model.ncnn.param`: a small text file describing the graph structure,
  - `model.ncnn.bin`: the binary weights.
- **Runtime:** the NCNN C++ library (with Python bindings).
- **Strengths:**
  - hand-written **NEON** kernels for ARM, tuned for exactly the kind of CPU in the Pi,
  - no third-party dependencies, small and fast to load,
  - aggressive graph optimisation and fusion at conversion time,
  - optional Vulkan GPU support (not useful on the Pi 5 in practice).
- **Weaknesses:** less portable than ONNX; less mainstream tooling; it supports a smaller set of operations (not a problem for YOLO, which is a well-supported case).
- **Why it matters here:** Ultralytics' own Raspberry Pi guide recommends NCNN as the fastest CPU option for YOLO on a Pi.

### 6.4 Other Formats You'll See (For Context)

| Format | From | Built for |
|---|---|---|
| TorchScript (`.torchscript`) | PyTorch | running a PyTorch graph without Python (still the PyTorch engine) |
| TFLite (`.tflite`) | Google | phones, microcontrollers, Coral Edge TPU |
| OpenVINO (`.xml` + `.bin`) | Intel | Intel CPUs, integrated GPUs and VPUs |
| TensorRT (`.engine`) | NVIDIA | NVIDIA GPUs (for example Jetson boards) — fastest there, unusable elsewhere |
| CoreML (`.mlpackage`) | Apple | iPhone and Mac Neural Engine |
| Hailo HEF (`.hef`) | Hailo | the Hailo AI accelerator (for example the Raspberry Pi AI Kit) |

The general pattern: **the more a format targets one specific type of hardware, the faster it tends to run on that hardware, and the less portable it is.** NCNN is the specialist for ARM CPUs, which is the hardware the rover has.

### 6.5 Summary Comparison

| | PyTorch `.pt` | ONNX | NCNN |
|---|---|---|---|
| Contains | Python object (pickle) | standard graph | ARM-optimised graph |
| Needs Python? | yes | no (but can use it) | no (but can use it) |
| Fixed input shape? | no | usually | yes |
| Portability | Python only | very high | ARM/mobile focus |
| Speed on Pi CPU | slowest | faster | usually fastest |
| Can be trained further? | yes | no | no |

---

## Part 7: What Changes and What Doesn't

### 7.1 What Stays the Same

- **The weights.** These are the same numbers, possibly rearranged by fusion.
- **The architecture.** It is mathematically the same function.
- **The detections.** The same boxes, classes and confidence scores, apart from tiny floating-point differences.

An export makes the model **faster, not smarter.** Accuracy, detection range and close-range clipping behaviour (§4 of the oscillation doc) are all unchanged.

### 7.2 Why "Tiny Floating-Point Differences"?

Computers store real numbers with limited precision (32-bit floats have about 7 significant digits). Rounding happens after every operation, so **the order of operations affects the last few digits.** For example, $(a + b) + c$ and $a + (b + c)$ can differ slightly.

Fusion and different kernels reorder the arithmetic, so the outputs differ from PyTorch's in maybe the 5th or 6th significant digit. In pixel terms, box coordinates move by a small fraction of a pixel. This is far below the detection jitter discussed in the oscillation doc.

### 7.3 What Lives Outside the Model

Not everything in `model.track()` is the neural network. Ultralytics splits each frame into three stages:

1. **Pre-processing (Python):** resize and pad the camera frame to 640×640 ("letterboxing") and convert it to a tensor.
2. **Model inference:** the neural network itself. **This is the only stage the export changes.**
3. **Post-processing (Python):** filter by confidence, run *non-maximum suppression* (NMS) to remove duplicate boxes, then run the **ByteTrack** tracker to assign track IDs across frames.

Because tracking happens in stage 3, outside the network, it works the same with an exported model, and the `Results` objects returned look the same.

---

## Part 8: Related Levers (Mentioned for Completeness)

### 8.1 Input Size (`imgsz`)

The camera captures 1920×1080, but YOLO always resizes the frame to the model's input size before inference (640×640 by default).

For convolutional networks, compute scales roughly with the **number of input pixels**:

$$
\frac{\text{cost}(640)}{\text{cost}(320)} \approx \frac{640^2}{320^2} = 4
$$

So exporting at `imgsz=320` would make inference roughly 4× cheaper again, on top of the format speed-up. The trade-off is that small or distant people cover fewer pixels and are harder to detect. Since the input size is fixed at export (Part 5.3), choosing it is part of the export decision.

### 8.2 Quantisation (FP32 → INT8)

By default, weights and activations are **32-bit floats (FP32)**. **Quantisation** stores them as **8-bit integers (INT8)** instead:

- 4× smaller model,
- integer maths is cheaper, and 4× more values fit in each SIMD register,
- typically another sizeable speed-up on CPUs,
- **but** it loses some precision, so accuracy drops slightly. It also needs a **calibration** step: a set of sample images is run through the model to choose the scaling ranges.

This is a separate, later optimisation. Export at FP32 first, measure, and only consider INT8 if more speed is still needed.

---

## Part 9: How This Relates to the Pan Oscillation Problem

### 9.1 Where Inference Sits in the Loop

From the oscillation investigation, each pipeline cycle takes about **0.45 s** (≈2.2 Hz), and it includes:

- LiDAR read,
- getting the camera frame,
- **YOLO inference** ← the only part the export speeds up,
- `results[0].plot()` at full 1920×1080 resolution,
- the JPEG encode for the MJPEG stream.

Using the notation from [control_loop_rate_theory.md](control_loop_rate_theory.md):

$$
T_{\text{cycle}} = T_{\text{inf}} + T_{\text{other}}
$$

where:

- **$T_{\text{inf}}$** is model inference time per cycle.
- **$T_{\text{other}}$** is everything else (camera, LiDAR, annotation, encoding, control, sleep).

### 9.2 Amdahl's Law — Why "2–3× Faster Model" ≠ "2–3× Faster Loop"

**Amdahl's law** says that speeding up one part of a process only helps in proportion to how much of the total time that part takes. If inference gets $s$ times faster:

$$
T_{\text{cycle,new}} = \frac{T_{\text{inf}}}{s} + T_{\text{other}}
$$

where:

- **$s$** is the inference speed-up factor (for example 2.5).
- **$T_{\text{cycle,new}}$** is the new cycle time.

**Worked example A: inference dominates.** Suppose $T_{\text{inf}} = 0.30$ s and $T_{\text{other}} = 0.15$ s, with $s = 2.5$:

$$
T_{\text{cycle,new}} = \frac{0.30}{2.5} + 0.15 = 0.12 + 0.15 = 0.27 \text{ s} \quad (\approx 3.7 \text{ Hz})
$$

**Worked example B: annotation and encoding dominate.** Suppose $T_{\text{inf}} = 0.15$ s and $T_{\text{other}} = 0.30$ s, with the same $s = 2.5$:

$$
T_{\text{cycle,new}} = \frac{0.15}{2.5} + 0.30 = 0.06 + 0.30 = 0.36 \text{ s} \quad (\approx 2.8 \text{ Hz})
$$

The same 2.5× faster model gives a 1.7× faster loop in case A, but only a 1.25× faster loop in case B.

*(The splits in both examples are illustrative. The real split hasn't been measured yet. Logging inference time per cycle is part of Step 1 of the oscillation doc.)*

So **measure $T_{\text{inf}}$ on its own before expecting a big win**, and expect to combine the export with the other Step 1 changes: moving the full-resolution `plot()` and the JPEG encode off the control path.

### 9.3 Why a Faster Cycle Helps Stability

The oscillation doc models the pan loop as:

$$
p[k+1] = p[k] + K \cdot (\theta - p[k-d])
$$

where **$d$** is the vision delay measured in *control cycles*. With $K \approx 0.83$, the loop is stable for $d = 1$ but unstable for $d = 2$.

Faster inference helps in two ways:

1. **Less processing delay.** The time from frame capture to pan command shrinks, so the error the controller acts on is fresher.
2. **More frequent decisions.** Each cycle is shorter, so the controller corrects more often, in smaller steps.

But note: the export alone doesn't fix the *double-counting* of the old error against the new base (Step 2 of the oscillation doc). A faster model reduces how much latency there is. Time-aligning the base with the frame (Step 2) makes the loop *tolerant* of whatever latency remains. Both are needed.

### 9.4 Goal From the Oscillation Doc

Step 1's target is **grab-to-command latency under ~150 ms at 5 Hz or faster**. At 5 Hz the whole cycle budget is 200 ms, so if PyTorch inference alone takes a large part of the current ~450 ms cycle, the target is hard to reach without a faster runtime, a smaller `imgsz`, or both.

---

## Part 10: Common Misconceptions

### Misconception 1: "Converting the model makes it more accurate."

Reality:

- the export keeps the same weights and maths, so the detections are effectively identical,
- only speed (and memory use) changes.

### Misconception 2: "If the model is 3× faster, my loop will be 3× faster."

Reality:

- only the inference part of the cycle speeds up (Amdahl's law, Part 9.2),
- annotation, encoding and I/O stay the same unless you change them too.

### Misconception 3: "ONNX and NCNN are different neural networks."

Reality:

- they are different *file formats and runtimes* for the same network,
- like saving the same document as `.docx` or `.pdf`: same content, different tools to open it.

### Misconception 4: "I can keep training the exported model."

Reality:

- exported graphs are inference-only (batch-norm is folded, training paths are removed),
- keep the `.pt` as the master copy, and re-export whenever you retrain or change `imgsz`.

### Misconception 5: "The Pi has a GPU, so PyTorch should use it."

Reality:

- PyTorch GPU acceleration needs CUDA (NVIDIA only) or a few other specific backends,
- the Pi 5's VideoCore GPU isn't one of them, so inference runs on the CPU.

---

## Part 11: Quick Reference Glossary

- **Architecture:** the structure of a neural network: its layers and how they connect.
- **Weights / parameters:** the learned numbers inside the layers.
- **Tensor:** a multi-dimensional array of numbers.
- **Training:** the phase in which weights are learned (needs gradients, usually on a GPU).
- **Inference:** using fixed weights to make predictions (what the rover does).
- **FLOP:** one floating-point operation (for example one multiply or one add).
- **CPU:** general-purpose processor with a few powerful cores.
- **GPU:** processor with thousands of simple cores, good at parallel maths.
- **CUDA:** NVIDIA's GPU programming platform; PyTorch's main GPU path.
- **ISA:** instruction set architecture, the machine language a CPU understands (ARM, x86).
- **ARM:** power-efficient ISA used in phones and Raspberry Pis.
- **x86:** ISA used in most desktop and laptop CPUs (Intel, AMD).
- **SIMD:** one instruction operating on several values at once.
- **NEON:** ARM's SIMD instruction set.
- **Cache:** small, fast memory next to the CPU core.
- **Memory bandwidth:** rate at which data moves between RAM and the CPU.
- **Framework:** library for building, training and running models (PyTorch).
- **Serialisation:** converting an object to bytes for saving or sending.
- **Pickle:** Python's built-in serialisation format; Python-only and code-dependent.
- **`.pt` file:** zip archive holding a pickled PyTorch model plus its weight tensors.
- **Eager execution:** running each operation immediately as Python reaches it.
- **Dispatcher:** PyTorch machinery that picks which kernel to run for each operation.
- **Kernel:** low-level routine that performs one operation on specific hardware.
- **Autograd:** PyTorch's automatic gradient calculation (needed for training only).
- **Computation graph:** a model described as operations (nodes) and tensors (edges).
- **Tracing:** capturing a graph by recording the operations of one example run.
- **Static shape:** tensor sizes fixed in advance, which allows ahead-of-time optimisation.
- **Constant folding:** pre-computing parts of the graph that depend only on constants.
- **Operator fusion:** merging consecutive operations into one kernel.
- **Batch-norm folding:** absorbing batch-norm into the preceding convolution's weights.
- **Inference runtime / engine:** program that executes an exported graph (ONNX Runtime, NCNN).
- **Export:** converting a trained model into a deployable graph format.
- **ONNX:** open, portable computation-graph format.
- **ONNX Runtime:** Microsoft's inference engine for ONNX files.
- **NCNN:** Tencent's inference framework optimised for ARM and mobile CPUs.
- **Letterboxing:** resizing an image to the model input size while padding to keep the aspect ratio.
- **NMS (non-maximum suppression):** removes overlapping duplicate boxes.
- **ByteTrack:** the multi-object tracker Ultralytics uses to assign track IDs.
- **Quantisation:** storing numbers at lower precision (for example INT8) for speed.
- **Amdahl's law:** overall speed-up is limited by the fraction of time the improved part takes.

---

## Part 12: Study Path (Recommended)

If these concepts are new, study in this order:

1. what a neural network is: layers, weights, tensors, and training vs inference,
2. CPU vs GPU, and why SIMD matters for neural network maths,
3. serialisation, pickle, and what a `.pt` file really contains,
4. eager execution and where its overhead comes from,
5. computation graphs, tracing and static shapes,
6. graph optimisations (work through the batch-norm folding maths by hand once),
7. formats vs runtimes (ONNX/ONNX Runtime, NCNN),
8. Amdahl's law applied to the rover's control loop.

This order builds from what a model *is*, to what hardware it runs on, to how the software around it decides how fast it runs.
