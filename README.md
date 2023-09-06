# Controllers C++ implementations
Linear object:
* Continuous:
$$
    \dot{x}(t) = Ax(t) + Bu(t)
$$
* Discrete: 
$$
    x_{k+1} = Ax_{k} + Bu_{k}
$$

## PID
$$
    u(t) = K_{p}\left(e(t) + \frac{1}{T_{i}}\int_{0}^{t}{e(\gamma)d\gamma} + T_{d} \frac{d}{dt}e(t) \right)
$$

### Anti-windup

## MPC