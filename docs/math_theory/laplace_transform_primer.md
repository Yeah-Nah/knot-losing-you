# Laplace Transform Primer

This note is written as a university-level primer for readers who are comfortable with maths, but new to control-oriented engineering maths.

## 1. Why We Use The Laplace Transform

Many dynamic systems are written as differential equations in time $t$. The Laplace transform maps those equations into algebraic equations in a complex variable $s$, which is usually easier to manipulate and interpret.

In short:

1. Time domain: derivatives and integrals in $t$.
2. $s$-domain: algebra in $s$.

This is especially useful for linear time-invariant (LTI) systems, where transfer functions and feedback relationships are naturally expressed in the $s$-domain.

## 2. Time-Domain Signals

A time-domain signal is any quantity written as a function of time, such as $x(t)$.

Examples in control:

- input command $u(t)$
- output/measurement $y(t)$
- reference $r(t)$
- error $e(t)=r(t)-y(t)$

Interpretation: a time-domain signal tells you how a quantity evolves with time (rise, delay, overshoot, settling, oscillation).

## 3. Laplace Transform Definition

For a signal $f(t)$ (typically for $t \ge 0$), the one-sided Laplace transform is:

$$
F(s)=\mathcal{L}\{f(t)\}=\int_0^{\infty} f(t)e^{-st}\,dt
$$

### Symbol Definitions For The Formula

- $f(t)$: time-domain signal being transformed.
- $F(s)$: Laplace transform of $f(t)$.
- $\mathcal{L}\{\cdot\}$: Laplace transform operator.
- $t$: real time variable.
- $s$: complex-frequency variable, $s=\sigma + j\omega$.
- $\sigma=\Re(s)$: real part of $s$ (growth/decay weighting).
- $\omega=\Im(s)$: imaginary part of $s$ (oscillatory frequency component).
- $j$: imaginary unit, $j^2=-1$.
- $e^{-st}$: exponential kernel used by the transform.
- $\int_0^{\infty}(\cdot)dt$: integration over non-negative time.

### What The $s$-Domain Actually Is

For someone new to engineering maths, the $s$-domain is best thought of as a different mathematical view of the same signal or system.

In the time domain, you ask:

- what is the value of the signal at each moment in time?
- how does it rise, fall, oscillate, or settle?

In the $s$-domain, you ask:

- what combination of decays and oscillations make up this signal?
- how does the system respond to different dynamic modes?
- what algebraic structure describes the system behaviour?

So the $s$-domain is not a different physical system. It is a transformed representation of the same physical behaviour.
It rewrites time-based behaviour into a form where decay and oscillation are encoded by the complex variable $s = \sigma + j\omega$.

The two parts of $s$ have intuitive roles:

- $\sigma$ controls exponential growth or decay
- $\omega$ controls oscillation frequency

That is why $s$ is sometimes called the complex-frequency variable: it combines "how fast something grows or decays" with "how fast it oscillates".

This matters because many signals and system responses can be understood as combinations of terms like:

$$
e^{st} = e^{(\sigma + j\omega)t} = e^{\sigma t} e^{j\omega t}
$$

Using Euler's formula, the oscillatory part can be related to sine and cosine terms, so this one expression captures both exponential change and oscillation.

For control and system analysis, that is extremely useful:

- differentiation in time becomes multiplication by $s$
- integration in time becomes division by $s$
- differential equations become algebraic equations
- system behaviour can be read from poles, zeros, and gain

In practice, you usually move to the $s$-domain not because it is more physical, but because it is easier to analyse.

## 4. Existence And Region Of Convergence (ROC)

The transform does not automatically exist for every $s$. It exists on values of $s$ for which the integral converges.

That set is called the region of convergence (ROC).

- ROC depends on how quickly $f(t)$ grows or decays.
- In control practice, we often work with rational transfer functions and use pole locations to infer behaviour and stability.

## 5. Core Properties (Most Used In Control)

For one-sided transforms, with zero initial conditions unless stated otherwise:

### 5.1 Linearity

$$
\mathcal{L}\{a f(t)+b g(t)\}=aF(s)+bG(s)
$$

Symbols:

- $a,b$: constants.
- $f(t),g(t)$: time-domain signals.
- $F(s),G(s)$: Laplace transforms of $f(t),g(t)$.

### 5.2 Time Derivative

$$
\mathcal{L}\left\{\frac{df}{dt}\right\}=sF(s)-f(0^+)
$$

Symbols:

- $\frac{df}{dt}$: first derivative in time.
- $f(0^+)$: initial value immediately after $t=0$.

With zero initial condition ($f(0^+)=0$), this becomes $\mathcal{L}\{df/dt\}=sF(s)$.

### 5.3 Time Integral

$$
\mathcal{L}\left\{\int_0^t f(\tau)\,d\tau\right\}=\frac{F(s)}{s}
$$

Symbols:

- $\tau$: dummy integration variable in time.
- $\int_0^t f(\tau)d\tau$: accumulated area of $f$ up to time $t$.

## 6. From Differential Equation To Transfer Function

A standard LTI model is:

$$
a_n\frac{d^n y}{dt^n}+a_{n-1}\frac{d^{n-1}y}{dt^{n-1}}+\cdots+a_0 y
=b_m\frac{d^m u}{dt^m}+\cdots+b_0 u
$$

With zero initial conditions, Laplace transform gives:

$$
\left(a_n s^n+a_{n-1}s^{n-1}+\cdots+a_0\right)Y(s)
=\left(b_m s^m+\cdots+b_0\right)U(s)
$$

So the transfer function is:

$$
P(s)=\frac{Y(s)}{U(s)}=\frac{b_m s^m+\cdots+b_0}{a_n s^n+a_{n-1}s^{n-1}+\cdots+a_0}
$$

### Symbol Definitions For Transfer Functions

- $u(t)$: input signal (cause).
- $y(t)$: output signal (effect).
- $U(s)$: Laplace transform of $u(t)$.
- $Y(s)$: Laplace transform of $y(t)$.
- $P(s)$: plant/system transfer function (input-to-output map in $s$-domain).
- $a_i,b_i$: real system coefficients from the differential equation.
- $n,m$: highest derivative orders of output and input.

## 7. Poles, Zeros, And Gain

Given a rational transfer function:

$$
P(s)=\frac{N(s)}{D(s)}=K\frac{\prod_{i=1}^{n_z}(s-z_i)}{\prod_{k=1}^{n_p}(s-p_k)}
$$

### Symbol Definitions

- $N(s)$: numerator polynomial.
- $D(s)$: denominator polynomial.
- $K$: scalar gain (overall scale factor).
- $z_i$: zeros (roots of $N(s)$ where $N(z_i)=0$).
- $p_k$: poles (roots of $D(s)$ where $D(p_k)=0$).
- $n_z$: number of zeros.
- $n_p$: number of poles.

### What They Mean Physically

1. Poles:
- Determine the system's natural modes.
- Strongly govern stability and transient behaviour.
- If any pole has positive real part, response grows and the system is unstable.
- Poles further left in the complex plane (more negative real part) usually imply faster decay.

2. Zeros:
- Shape how input passes through to output.
- Influence transient shape, overshoot, and frequency response.
- Can attenuate or accentuate specific dynamic effects.

3. Gain $K$:
- Scales the response magnitude.
- Does not move pole locations by itself in a fixed transfer function, but changes loop behaviour when used inside feedback designs.

## 8. How Poles, Zeros, And Gain Fit Into $s$

The variable $s=\sigma+j\omega$ defines the complex plane (the $s$-plane).

- Pole and zero locations are points in that plane.
- Their locations encode dynamic behaviour.
- Real part ($\sigma$) links to growth/decay.
- Imaginary part ($\omega$) links to oscillation frequency.

A pole pair $p=\sigma\pm j\omega_d$ typically maps to a damped oscillatory mode in time:

$$
e^{\sigma t}\cos(\omega_d t),\quad e^{\sigma t}\sin(\omega_d t)
$$

So:

- $\sigma<0$: decaying oscillation (stable mode).
- $\sigma=0$: sustained oscillation (marginal mode).
- $\sigma>0$: growing oscillation (unstable mode).

## 9. Standard Control Quantities In The $s$-Domain

Common definitions:

$$
E(s)=R(s)-Y(s),\qquad U(s)=C(s)E(s),\qquad Y(s)=P(s)U(s)
$$

Closed-loop reference-to-output transfer function:

$$
\frac{Y(s)}{R(s)}=\frac{C(s)P(s)}{1+C(s)P(s)}
$$

### Symbol Definitions

- $r(t)$: reference signal in time domain.
- $R(s)$: Laplace transform of $r(t)$.
- $e(t)$: error signal in time domain.
- $E(s)$: Laplace transform of $e(t)$.
- $C(s)$: controller transfer function.
- $C(s)P(s)$: open-loop transfer function (loop gain).
- $\frac{Y(s)}{R(s)}$: closed-loop transfer function.

## 10. Practical Reading Guide

When you see an $s$-domain model:

1. Identify poles from the denominator.
2. Check pole real parts for stability intuition.
3. Inspect zeros for transient/frequency-shaping effects.
4. Track gain for magnitude scaling.
5. Use closed-loop forms to understand feedback effects.

This workflow connects engineering notation to the mathematical tools you already know from university-level analysis.
