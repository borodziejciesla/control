[![CodeFactor](https://www.codefactor.io/repository/github/borodziejciesla/control/badge)](https://www.codefactor.io/repository/github/borodziejciesla/control)

# Controllers C++ implementations
Linear object:
* Continuous: $\dot{x}(t) = Ax(t) + Bu(t)$
* Discrete: $x_{k+1} = Ax_{k} + Bu_{k}$

## PID
Cntinuous form:
$$
    u(s) = K_{p}\left(1 + T_{i}s + T_{d} + T_{d}\frac{s}{1 + s\frac{T_{d}}{N}} \right)e(s)
$$

Discrete form:
$$
    u(z) = K_{p} \left(1 + T_{i} \frac{z - 1}{T} + T_{d}\frac{\frac{z-1}{T}}{1+\frac{z-1}{T}\frac{T_{d}}{N}} \right) e(z)
$$

### Anti-windup

## MPC