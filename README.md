# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies and Basic Build Instructions

See [https://github.com/udacity/CarND-PID-Control-Project/blob/master/README.md]()

## PID Control

The core functionality is basically the PID::UpdateError function:
[https://github.com/balintfodor/CarND-PID-Control-Project/blob/master/src/PID.cpp#L24]()

```
void PID::UpdateError(double cte) {
    sum_cte += cte; // cumulative error
    p_error = Kp * cte; // Proporitonal term
    i_error = Ki * sum_cte; // Integral term
    d_error = Kd * (cte - prev_cte); // Derivative term
    prev_cte = cte;
}
```

The absolute error (**fabs(cte)**) is small if the car is on track and starts to grow when the car starts to leave the center of the track. **Cte** is a signed value indicating the side the car started to leave the track on.

The **P** term tries to directly compensate the error by steering to the oposite direction the absolute error grows. The magnitude of the compensation is proportional to the error value. Using only the P term to control the car can be seen in the following gif:

![](assets/only_p.gif)

As the speed increases the amplitude of the oscillation grows. The higher the absolute error the stronger it tries to compensate with steering. Also, the drove distance between two control timestamps (subsequent calls of PID::UpdateError) is also increasing (with the speed) and the overshoothing in the error grows too, ending in an unstable car.

The **D** component is for short term compensations. It reacts on the derivative of the error. So a huge change in the error will cause a rather large compensation. On the long term though, using only the **D** component can end up in a growing offset.

![](assets/only_d.gif)

Component **I** acts in the long term. It uses the cumulative error to steer the car back on track. Its effect is somewhat delayed since it takes time to integrate enough amount of error to react on. So, using only the **I** term will not give quick reactions.

![](assets/only_i.gif)

