import time
import serial
import json
import matplotlib.pyplot as pp
import numpy as np
import tprofile as tp

def rpc_call(method, params, dev = '/dev/cu.usbserial-A5026YP3'):
    request = {
        "id" : "noid", 
        "jsonrpc" : "1.0", 
        "method" : method, 
        "params" : params
    }
    requeststr = json.dumps(request) + '\n'
    ser = serial.Serial(
        port=dev,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=30
    )

    ser.isOpen()
    ser.write(requeststr.encode('utf-8'))
    ser.flush()
    responsestr = ser.readline().decode("utf-8")
    response = json.loads(responsestr)
    if 'error' in response:
        raise Exception("RPC Error: " + response['error']['message'])
    return response

class drive:
    def __init__(self, name, device = '/dev/cu.usbserial-A5026YP3'):
        self.namestr = name
        self.device = device


    def getcfg(self, name):
        return rpc_call("getcfg", [self.namestr + ".drive." + name], self.device)["result"]
    def setcfg(self, name, value):
        rpc_call("setcfg", [self.namestr + ".drive." + name, value], self.device)
    def get(self, name):
        return rpc_call("get", [self.namestr + ".drive." + name], self.device)["result"]
    def set(self, name, value):
        rpc_call("set", [self.namestr + ".drive." + name, value], self.device)
    def call(self, name, args):
        return rpc_call(self.namestr + ".drive." + name, args, self.device)["result"]

    @property
    def name(self):
        return self.namestr
    @name.setter
    def name(self, name):
        self.namestr = name

    #
    # Props
    #
    @property
    def Rencest(self):
        return self.get("Rencest")
    @Rencest.setter
    def Rencest(self, v):
        self.set("Rencest", v)

    @property
    def TIM1_CNT(self):
        return self.get("TIM1_CNT")
    @TIM1_CNT.setter
    def TIM1_CNT(self, v):
        self.set("TIM1_CNT", v)

    @property
    def TIM8_CNT(self):
        return self.get("TIM8_CNT")
    @TIM8_CNT.setter
    def TIM8_CNT(self, v):
        self.set("TIM8_CNT", v)

    @property
    def Vbus(self):
        return self.get("Vbus")
    @Vbus.setter
    def Vbus(self, v):
        self.set("Vbus", v)

    @property
    def error(self):
        return self.get("error")
    @error.setter
    def error(self, v):
        self.set("error", v)

    @property
    def error_msg(self):
        return self.get("error_msg")
    @error_msg.setter
    def error_msg(self, v):
        self.set("error_msg", v)

    @property
    def lpf_bias_a(self):
        return self.get("lpf_bias_a")
    @lpf_bias_a.setter
    def lpf_bias_a(self, v):
        self.set("lpf_bias_a", v)

    @property
    def lpf_bias_b(self):
        return self.get("lpf_bias_b")
    @lpf_bias_b.setter
    def lpf_bias_b(self, v):
        self.set("lpf_bias_b", v)

    @property
    def lpf_bias_c(self):
        return self.get("lpf_bias_c")
    @lpf_bias_c.setter
    def lpf_bias_c(self, v):
        self.set("lpf_bias_c", v)

    @property
    def lpf_bias_a(self):
        return self.get("lpf_bias_a")
    @lpf_bias_a.setter
    def lpf_bias_a(self, v):
        self.set("lpf_bias_a", v)

    @property
    def tim1_cnt(self):
        return self.get("tim1_cnt")
    @tim1_cnt.setter
    def tim1_cnt(self, v):
        self.set("tim1_cnt", v)

    @property
    def tim1_tim8_offset(self):
        return self.get("tim1_tim8_offset")
    @tim1_tim8_offset.setter
    def tim1_tim8_offset(self, v):
        self.set("tim1_tim8_offset", v)

    @property
    def tim8_cnt(self):
        return self.get("tim8_cnt")
    @tim8_cnt.setter
    def tim8_cnt(self, v):
        self.set("tim8_cnt", v)

    @property
    def tim8_tim1_offset(self):
        return self.get("tim8_tim1_offset")
    @tim8_tim1_offset.setter
    def tim8_tim1_offset(self, v):
        self.set("tim8_tim1_offset", v)

    @property
    def time_slice(self):
        return self.get("time_slice")
    @time_slice.setter
    def time_slice(self, v):
        self.set("time_slice", v)

    @property
    def update_hz(self):
        return self.get("update_hz")
    @update_hz.setter
    def update_hz(self, v):
        self.set("update_hz", v)


    #
    # Config Props
    #
    @property
    def bias_alpha(self):
        return self.getcfg("bias_alpha")
    @bias_alpha.setter
    def bias_alpha(self, v):
        self.setcfg("bias_alpha", v)

    @property
    def calib_max_i(self):
        return self.getcfg("calib_max_i")
    @calib_max_i.setter
    def calib_max_i(self, v):
        self.setcfg("calib_max_i", v)

    @property
    def calib_v(self):
        return self.getcfg("calib_v")
    @calib_v.setter
    def calib_v(self, v):
        self.setcfg("calib_max_i", v)

    @property
    def csa_gain(self):
        return self.getcfg("csa_gain")
    @csa_gain.setter
    def csa_gain(self, v):
        self.setcfg("csa_gain", v)

    @property
    def display_div(self):
        return self.getcfg("display_div")
    @display_div.setter
    def display_div(self, v):
        self.setcfg("display_div", v)

    @property
    def enc_skip_updates(self):
        return self.getcfg("enc_skip_updates")
    @enc_skip_updates.setter
    def enc_skip_updates(self, v):
        self.setcfg("enc_skip_updates", v)

    @property
    def encoder_dir(self):
        return self.getcfg("encoder_dir")
    @encoder_dir.setter
    def encoder_dir(self, v):
        self.setcfg("encoder_dir", v)

    @property
    def inductance(self):
        return self.getcfg("inductance")
    @inductance.setter
    def inductance(self, v):
        self.setcfg("inductance", v)

    @property
    def max_modulation_duty(self):
        return self.getcfg("max_modulation_duty")
    @max_modulation_duty.setter
    def max_modulation_duty(self, v):
        self.setcfg("max_modulation_duty", v)

    @property
    def pole_pairs(self):
        return self.getcfg("pole_pairs")
    @pole_pairs.setter
    def pole_pairs(self, v):
        self.setcfg("pole_pairs", v)

    @property
    def reset_hz(self):
        return self.getcfg("reset_hz")
    @reset_hz.setter
    def reset_hz(self, v):
        self.setcfg("reset_hz", v)

    @property
    def reset_voltage(self):
        return self.getcfg("reset_voltage")
    @reset_voltage.setter
    def reset_voltage(self, v):
        self.setcfg("reset_voltage", v)

    @property
    def resistance(self):
        return self.getcfg("resistance")
    @resistance.setter
    def resistance(self, v):
        self.setcfg("resistance", v)

    @property
    def run_simple_tasks(self):
        return self.getcfg("run_simple_tasks")
    @run_simple_tasks.setter
    def run_simple_tasks(self, v):
        self.setcfg("run_simple_tasks", v)

    @property
    def svm_saddle(self):
        return self.getcfg("svm_saddle")
    @svm_saddle.setter
    def svm_saddle(self, v):
        self.setcfg("svm_saddle", v)

    @property
    def trip_i(self):
        return self.getcfg("trip_i")
    @trip_i.setter
    def trip_i(self, v):
        self.setcfg("trip_i", v)

    @property
    def trip_v(self):
        return self.getcfg("trip_v")
    @trip_v.setter
    def trip_v(self, v):
        self.setcfg("trip_v", v)

    @property
    def vbus_alpha(self):
        return self.getcfg("vbus_alpha")
    @vbus_alpha.setter
    def vbus_alpha(self, v):
        self.setcfg("vbus_alpha", v)

    @property
    def wenc_alpha(self):
        return self.getcfg("wenc_alpha")
    @wenc_alpha.setter
    def wenc_alpha(self, v):
        self.setcfg("wenc_alpha", v)

    def abort(self):
        return self.call("abort", [])
    def add_task_arm_motor(self):
        return self.call("add_task_arm_motor", [])
    def add_task_disarm_motor(self):
        return self.call("add_task_disarm_motor", [])
    def add_task_reset_rotor(self):
        return self.call("add_task_reset_rotor", [])
    def add_task_rotate_motor(self, angle, speed, voltage, dir):
        return self.call("add_task_rotate_motor", [angle, speed, voltage, dir])
    def alpha_pole_search(self):
        return self.call("alpha_pole_search", [])
    def drv_clear_fault(self):
        return self.call("drv_clear_fault", [])
    def drv_get_fault1(self):
        return self.call("drv_get_fault1", [])
    def drv_get_fault2(self):
        return self.call("drv_get_fault2", [])
    def measure_inductance(self, seconds, test_voltage, test_hz):
        return self.call("measure_inductance", [seconds, test_voltage, test_hz])
    def measure_resistance(self, seconds, test_voltage):
        return self.call("measure_resistance", [seconds, test_voltage])
    def rotate(self, angle, speed, voltage, dir):
        return self.call("rotate", [angle, speed, voltage, dir])
    def run_encoder_debug(self):
        return self.call("run_encoder_debug", [])
    def scheduler_abort(self):
        return self.call("scheduler_abort", [])
    def scheduler_run(self):
        return self.call("scheduler_run", [])
    def velocitypts(self):
        return self.call("velocitypts", [])



class axis:
    def __init__(self, name, device = '/dev/cu.usbserial-A5026YP3'):
        self.namestr = name
        self.drive = drive(name, device)
        self.device = device



    def getcfg(self, name):
        return rpc_call("getcfg", [self.namestr + "." + name], self.device)["result"]
    def setcfg(self, name, value):
        rpc_call("setcfg", [self.namestr + "." + name, value], self.device)
    def get(self, name):
        return rpc_call("get", [self.namestr + "." + name], self.device)["result"]
    def set(self, name, value):
        rpc_call("set", [self.namestr + "." + name, value], self.device)
    def call(self, name, args):
        return rpc_call(self.namestr + "." + name, args, self.device)["result"]

    @property
    def name(self):
        return self.namestr
    @name.setter
    def name(self, name):
        self.namestr = name

    #
    # Props
    #
    @property
    def velocity(self):
        return self.get("velocity")
    @velocity.setter
    def velocity(self, v):
        self.set("velocity", v)

    @property
    def target(self):
        return self.get("target")
    @target.setter
    def target(self, v):
        self.set("target", v)

    @property
    def spin_voltage(self):
        return self.get("spin_voltage")
    @spin_voltage.setter
    def spin_voltage(self, v):
        self.set("spin_voltage", v)

    @property
    def q_current(self):
        return self.get("q_current")
    @q_current.setter
    def q_current(self, v):
        self.set("q_current", v)

    @property
    def acceleration(self):
        return self.get("acceleration")
    @acceleration.setter
    def acceleration(self, v):
        self.set("acceleration", v)

    @property
    def deceleration(self):
        return self.get("deceleration")
    @deceleration.setter
    def deceleration(self, v):
        self.set("deceleration", v)

    @property
    def capture_mode(self):
        return self.get("capture_mode")
    @capture_mode.setter
    def capture_mode(self, v):
        self.set("capture_mode", v)

    @property
    def capture_interval(self):
        return self.get("capture_interval")
    @capture_interval.setter
    def capture_interval(self, v):
        self.set("capture_interval", v)

    @property
    def capture_capacity(self):
        return self.get("capture_capacity")
    @capture_capacity.setter
    def capture_capacity(self, v):
        self.set("capture_capacity", v)

    #
    # Config Props
    #
    @property
    def w_bias(self):
        return self.getcfg("w_bias")
    @w_bias.setter
    def w_bias(self, v):
        self.setcfg("w_bias", v)

    @property
    def vab_advance_factor(self):
        return self.getcfg("vab_advance_factor")
    @vab_advance_factor.setter
    def vab_advance_factor(self, v):
        self.setcfg("vab_advance_factor", v)

    @property
    def tau_ratio(self):
        return self.getcfg("tau_ratio")
    @tau_ratio.setter
    def tau_ratio(self, v):
        self.setcfg("tau_ratio", v)

    @property
    def pid_w_maxout(self):
        return self.getcfg("pid_w_maxout")
    @pid_w_maxout.setter
    def pid_w_maxout(self, v):
        self.setcfg("pid_w_maxout", v)

    @property
    def pid_w_kp(self):
        return self.getcfg("pid_w_kp")
    @pid_w_kp.setter
    def pid_w_kp(self, v):
        self.setcfg("pid_w_kp", v)

    @property
    def pid_w_ki(self):
        return self.getcfg("pid_w_ki")
    @pid_w_ki.setter
    def pid_w_ki(self, v):
        self.setcfg("pid_w_ki", v)

    @property
    def pid_p_maxout(self):
        return self.getcfg("pid_p_maxout")
    @pid_p_maxout.setter
    def pid_p_maxout(self, v):
        self.setcfg("pid_p_maxout", v)

    @property
    def pid_p_kp(self):
        return self.getcfg("pid_p_kp")
    @pid_p_kp.setter
    def pid_p_kp(self, v):
        self.setcfg("pid_p_kp", v)

    @property
    def pid_current_maxout(self):
        return self.getcfg("pid_current_maxout")
    @pid_current_maxout.setter
    def pid_current_maxout(self, v):
        self.setcfg("pid_current_maxout", v)

    @property
    def pid_current_kp(self):
        return self.getcfg("pid_current_kp")
    @pid_current_kp.setter
    def pid_current_kp(self, v):
        self.setcfg("pid_current_kp", v)

    @property
    def pid_current_ki(self):
        return self.getcfg("pid_current_ki")
    @pid_current_ki.setter
    def pid_current_ki(self, v):
        self.setcfg("pid_current_ki", v)

    @property
    def display(self):
        return self.getcfg("display")
    @display.setter
    def display(self, v):
        self.setcfg("display", v)

    def calibration(self, reset_rotor):
        return self.call("calibration", [reset_rotor])
    def velocity_rps(self):
        return self.call("velocity_rps", [])
    def stop(self):
        return self.call("stop", [])
    def modeclp(self):
        return self.call("modeclp", [])
    def modeclps(self):
        return self.call("modeclps", [])
    def modeclv(self):
        return self.call("modeclv", [])
    def modespin(self):
        return self.call("modespin", [])
    def modeclt(self):
        return self.call("modeclt", [])
    def go(self):
        return self.call("go", [])
    def get_captured_velocityspec(self):
        return self.call("get_captured_velocityspec", [])
    def get_captured_velocity(self):
        return self.call("get_captured_velocity", [])
    def get_captured_position(self):
        return self.call("get_captured_position", [])
    def get_captured_current(self):
        return self.call("get_captured_current", [])
    def get_sequence(self, count):
        return self.call("get_sequence", [count])
    def mvp(self, pos):
        return self.call("mvp", [pos])
    def mvr(self, pos):
        return self.call("mvr", [pos])
    def push(self, time, velocity, position):
        return self.call("push", [time, velocity, position])
    def go(self):
        return self.call("go", [])

    def capture(self):
        V = self.get_captured_velocity()
        S = self.get_captured_velocityspec()
        P = self.get_captured_position()

        pp.figure()
        pp.subplot(2,1,1)
        pp.plot(np.arange(0, len(S)), S, alpha=0.75, linewidth=3, label="Spec")
        pp.plot(np.arange(0, len(V)), V, label="Velocity")
        pp.ylabel('Velocity')

        pp.subplot(2,1,2)
        pp.plot(np.arange(0, len(P)), P, color="orange", linewidth=3, label="Position")
        pp.xlabel('Time')
        pp.ylabel('Position')
        pp.show()

def mvrx(axis, P, V, A, D):
    curtarget = axis.target
    newtarget = curtarget + P
    points = tp.TrapezoidalProfile(curtarget, newtarget, 0, 0, V, A, D, axis.drive.update_hz)
    for i in range(0, len(points)):
        axis.push(int(points[i][0]), points[i][1], points[i][2])
    axis.target = newtarget
    axis.go()
    return points

