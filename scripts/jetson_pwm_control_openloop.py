import Jetson.GPIO as GPIO
import time
import rospy

from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

PWM_PIN_TH = 32
PWM_PIN_ST = 33

duty_cycle_th = 15.0 # 15 for 0% throttle
duty_cycle_st = 15.0 # 15 for 0% steering

DUTY_CYCLE_TH_BIAS = 0.0 # for throttle bias
DUTY_CYCLE_ST_BIAS = 0.0 # for steering

DUTY_CYCLE_TH_CENTER_THEORY = 15.0 # 15 for 0% throttle, const
DUTY_CYCLE_ST_CENTER_THEORY = 15.0 # 15 for 0% throttle, const

DUTY_CYCLE_ST_VAR_MAX = 20.0
DUTY_CYCLE_ST_VAR_MIN = 10.0
DUTY_CYCLE_TH_VAR_MAX = 20.0
DUTY_CYCLE_TH_VAR_MIN = 10.0

L = 0.20 # m for axle length
VR_THRESHOLD = 0.05 # m/s for threshold of linear velocity


cmd_twist = None
odom_twist = Odometry()
odom_twist.twist = TwistWithCovariance()
odom_twist.twist.twist.angular.z = 0.
odom_twist.twist.twist.linear.x = 0.


def callback_odom(msg):
    '''
    copy the twist from odom message, called from callback
    '''
    global odom_twist
    odom_twist = msg.twist.twist

def callback_cmd_vel(msg):
    '''
    copy the twist from cmd_vel message, called from callback
    '''
    global cmd_twist
    cmd_twist = msg

def callback_control(event):
    '''
    generate control variable and apply to hardware pin.
    from the error of cmd_vel and odom,
    main steps are:
    1.after trans odom to control var space,
    2.using PID control to generate control var.
    3.transform control var to duty cycle and apply to hardware pin.
    '''
    global duty_cycle_th, duty_cycle_st
    global cmd_twist, odom_twist
    global DUTY_CYCLE_TH_BIAS, DUTY_CYCLE_ST_BIAS
    global DUTY_CYCLE_TH_CENTER_THEORY, DUTY_CYCLE_ST_CENTER_THEORY
    global L, VR_THRESHOLD
    global DUTY_CYCLE_TH_VAR_MAX, DUTY_CYCLE_TH_VAR_MIN, DUTY_CYCLE_ST_VAR_MAX, DUTY_CYCLE_ST_VAR_MIN

    # 1. trans to control var space
    v = odom_twist.linear.x
    w = odom_twist.angular.z

    vr = v

    if math.abs(vr) < VR_THRESHOLD:
        delta = 0. # in case of infinite value
    else:
        delta = math.atan2(w*L, vr)  #delta for steering angle

    vc = cmd_twist.linear.x
    wc = cmd_twist.angular.z

    vcr = vc

    if math.abs(vcr) < VR_THRESHOLD:
        delta_c = 0.
    else:
        delta_c = math.atan2(wc*L, vcr)

    # 2. PID control
    delta_err = delta_c - delta
    v_err = vcr - vr

    Kp_delta = 0.01
    Kp_v = 0.01
    Ki = 0.0
    Kd = 0.0

    delta_control = Kp_delta * delta_err 
    v_control = Kp_v * v_err 

    # 3. transform to duty cycle

    duty_cycle_th = np.clip(DUTY_CYCLE_TH_CENTER_THEORY + DUTY_CYCLE_TH_BIAS + v_control, DUTY_CYCLE_TH_VAR_MIN, DUTY_CYCLE_TH_VAR_MAX)
    duty_cycle_st = np.clip(DUTY_CYCLE_ST_CENTER_THEORY + DUTY_CYCLE_ST_BIAS + delta_control, DUTY_CYCLE_ST_VAR_MIN, DUTY_CYCLE_ST_VAR_MAX)

def change_pwm_duty_cycle(event, pwm_handler: GPIO.PWM, pwm_duty_cycle: float):
    '''
    change the duty cycle of pwm handler
    '''
    pwm_handler.ChangeDutyCycle(pwm_duty_cycle)

if __name__ == "__main__":
    rospy.init_node("jetson_pwm_control_node")
    sub_odom = rospy.Subscriber("/odom", Odometry, callback_odom)
    sub_cmd_vel = rospy.Subscriber("/cmd_vel", Twist, callback_cmd_vel)
    timer = rospy.Timer(rospy.Duration(0.01), callback_control)

    pwm_pin_th = PWM_PIN_TH
    pwm_pin_st = PWM_PIN_ST

    GPIO.setup(pwm_pin_th, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(pwm_pin_st, GPIO.OUT, initial=GPIO.LOW)

    pwm_st = GPIO.PWM(pwm_pin_st, 100)
    pwm_th = GPIO.PWM(pwm_pin_th, 100)

    print("[+] starting pwm wave... ")
    pwm_st.start(DUTY_CYCLE_ST_CENTER_THEORY + DUTY_CYCLE_ST_BIAS)
    pwm_th.start(DUTY_CYCLE_TH_CENTER_THEORY + DUTY_CYCLE_TH_BIAS)

    timer_st = rospy.Timer(rospy.Duration(0.01), change_pwm_duty_cycle, pwm_st, duty_cycle_st)
    timer_th = rospy.Timer(rospy.Duration(0.01), change_pwm_duty_cycle, pwm_th, duty_cycle_th)

    try:
        rospy.spin()
        while not rospy.is_shutdown():
            pass
    except Exception as e:
        rospy.ERROR("[E] error: {}".format(e))
    finally:
        pwm_st.stop()
        pwm_th.stop()
        GPIO.cleanup()
        rospy.INFO("[I] jetson_pwm_control_node is shutdown")







