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

## Manual Tuning

I tuned my coefficients as [the introduction by George Gillard](http://smithcsrobot.weebly.com/uploads/6/0/9/5/60954939/pid_control_document.pdf):

1. Set Kp, Ki, and Kd to 0. This will disable them for now.
2. Increase Kp until the error is fairly small, but it still gets from the beginning to nearly
the end quickly enough.
3. Increase Kd until any overshoot you may have is fairly minimal. But be careful with
Kd – too much will make it overshoot.
4. Increase Ki until any error that is still existing is eliminated. Start with a really small
number for Ki, don't be surprised if it is as small as 0.0001 or even smaller.
5. Using the rules of tuning the constants (in the table on the previous page), you can
change around the constants a little bit to get it working to the best performance.

I final tuning result is `Kp` = 4, `Ki` = 0.002, `Kd` = 0.02. I also reset the `throttle` as 0.2.


## Throttle

As a common sense, we should slow down vehilce speed when turning sharply. Thus I add the code as below:

```
double throttle_value = 0.2;
if(speed > 1) {
  // decrease speed when steer_value > 5
  throttle_value = 0.2 * (5 - fabs(steer_value))/5;
}
```
`throttle_value = 0.2 * (5 - fabs(steer_value))/5;` means when steer\_value is bigger than 5 degree, throttle is negative. That is to say vehicle speed will be slowed down when steer_value is more than 5 degree. And the closer the steer value is to 25 degree, the faster the speed is reduced.

Condition `speed > 1` prevents the vehicle stopping or reversing.
