[//]: # (Image References)

[image0]: ./ReadmeImages/plot.png "Flowchar"
[image1]: ./ReadmeImages/PID.png "yaw_rate!=0"
[image2]: ./ReadmeImages/twiddle.png 
[image3]: ./ReadmeImages/ParticleWeights.png 


# PID Controller

## Why PID Controller?

The cross track error, CTE is the current y position of the vehicle (our reference is a horizontal line) along the x-axis.

![alt text][image0]

From the plot above we can see that:

- For P controller, the turning variable of steering angle in unit time equals a proportional factor of CTE, the car will overshot.

- For PD controller, the turning variable of the steering angle is not only related the current CTE  but also to the temporal derivative of the CTE. But differential term, or D-term, can't solve the systematic bias problem.

- For PID controller, besides the current CTE and the temporal derivative of CTE, it's also proportional to the integral or the sum of all the CTE you ever observed.

## `PID.cpp`

![alt text][image1]

## Twiddle

This picture shows how twiddle works.

![alt text][image2]

I set the initialization of the best error equals to the cte after a unit time if the steering angle kept unchanged.

Note: in twiddle method, error is the horizontal distance to the reference trajectory in the vehicle coordinate system.


