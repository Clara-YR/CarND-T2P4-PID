[//]: # (Image References)

[image0]: ./ReadmeImages/plot.png "Flowchar"
[image1]: ./ReadmeImages/PID.png "yaw_rate!=0"
[image2]: ./ReadmeImages/PID_formula.png "yaw_rate!=0"
[image3]: ./ReadmeImages/twiddle.png 
[image4]: ./ReadmeImages/vehicle_coordinate.png 


# PID Controller

## Why PID Controller?

The cross track error, CTE is the current y position of the vehicle (our reference is a horizontal line) along the x-axis.

![alt text][image0]

From the plot above we can see that:

- For P controller, the turning variable of steering angle in unit time equals a proportional factor of CTE, the car will overshot.

- For PD controller, the turning variable of the steering angle is not only related the current CTE  but also to the temporal derivative of the CTE. But differential term, or D-term, can't solve the systematic bias problem.

- For PID controller, besides the current CTE and the temporal derivative of CTE, it's also proportional to the integral or the sum of all the CTE you ever observed.

## PID Calculation

![alt text][image1]
![alt text][image2]

## Twiddle

This picture shows how twiddle works.

![alt text][image3]

I set the initialization of the best error equals to the cte after a unit time if the steering angle kept unchanged.

Note: in twiddle method, error is the distance bewteen the vehicle position and the reference trajectory in y direction as below. 

![alt text][image4]

Compare the __absolute__ value of `best_error` and `error`.


