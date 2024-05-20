from controller import Robot
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctl

def run_robot(robot):

    #define
    time_step = 32
    max_speed = 6.28
    
    # Input
    right_ir_a = ctl.Antecedent(np.arange(0, 1001, 1), 'Right a')
    right_ir_b = ctl.Antecedent(np.arange(0, 1001, 1), 'Right b')
    mid_ir_a = ctl.Antecedent(np.arange(0, 1001, 1), 'Mid a')
    mid_ir_b = ctl.Antecedent(np.arange(0, 1001, 1), 'Mid b')
    mid_ir_c = ctl.Antecedent(np.arange(0, 1001, 1), 'Mid c')
    left_ir_a = ctl.Antecedent(np.arange(0, 1001, 1), 'Left a')
    left_ir_b = ctl.Antecedent(np.arange(0, 1001, 1), 'Left b')

    # Output
    rms = ctl.Consequent(np.arange(0, 101, 1), 'Kecepatan Motor Kanan')
    lms = ctl.Consequent(np.arange(0, 101, 1), 'Kecepatan Motor Kiri')

    # mf sensor
    for sensor in [right_ir_a, right_ir_b, mid_ir_a, mid_ir_b, mid_ir_c, left_ir_a, left_ir_b]:
        sensor['rendah'] = fuzz.trimf(sensor.universe, [0, 0, 500])
        sensor['sedang'] = fuzz.trimf(sensor.universe, [0, 500, 1000])
        sensor['tinggi'] = fuzz.trimf(sensor.universe, [500, 1000, 1000])

    # mf PWM motor
    rms['lambat'] = fuzz.trimf(rms.universe, [0, 0, 50])
    rms['sedang'] = fuzz.trimf(rms.universe, [0, 50, 100])
    rms['cepat'] = fuzz.trimf(rms.universe, [50, 100, 100])

    lms['lambat'] = fuzz.trimf(lms.universe, [0, 0, 50])
    lms['sedang'] = fuzz.trimf(lms.universe, [0, 50, 100])
    lms['cepat'] = fuzz.trimf(lms.universe, [50, 100, 100])

    #motor
    lms = robot.getDevice('left wheel motor')
    rms = robot.getDevice('right wheel motor')
    lms.setPosition(float('inf'))
    rms.setPosition(float('inf'))
    lms.setVelocity(0.0)
    rms.setVelocity(0.0)

    #ir
    r_ir_a = robot.getDevice('ir0')
    r_ir_a.enable(time_step)
    r_ir_b = robot.getDevice('ir1')
    r_ir_b.enable(time_step)
    m_ir_a = robot.getDevice('ir2')
    m_ir_a.enable(time_step)
    m_ir_b = robot.getDevice('ir3')
    m_ir_b.enable(time_step)
    m_ir_c = robot.getDevice('ir4')
    m_ir_c.enable(time_step)
    l_ir_a = robot.getDevice('ir5')
    l_ir_a.enable(time_step)
    l_ir_b = robot.getDevice('ir6')
    l_ir_b.enable(time_step)

    while robot.step(time_step) != -1:

        #pembacaan ir
        right_ir_a_value = r_ir_a.getValue()
        right_ir_b_value = r_ir_b.getValue()
        mid_ir_a_value = m_ir_a.getValue()
        mid_ir_b_value = m_ir_b.getValue()
        mid_ir_c_value = m_ir_c.getValue()
        left_ir_a_value = l_ir_a.getValue()
        left_ir_b_value = l_ir_b.getValue()

        print('Left a {} Left b {}  Mid a {} Mid b {} Mid c {} Right a {} Right b {}'.format(right_ir_a_value, right_ir_b_value, mid_ir_a_value, mid_ir_b_value, mid_ir_c_value, left_ir_a_value, left_ir_b_value))

        left_speed = -max_speed
        right_speed = -max_speed
        
        rule1 = ctl.Rule(right_ir_a['rendah'] & right_ir_b['rendah'] & mid_ir_a['rendah'] & mid_ir_b['rendah'] & mid_ir_c['rendah'] & left_ir_a['rendah'] & left_ir_b['rendah'], [rms['lambat'], lms['cepat']])

        rule2 = ctl.Rule(right_ir_a['tinggi'] & right_ir_b['tinggi'] & mid_ir_a['tinggi'] & mid_ir_b['tinggi'] & mid_ir_c['tinggi'] & left_ir_a['tinggi'] & left_ir_b['tinggi'], [rms['cepat'], lms['lambat']])

        rule3 = ctl.Rule(right_ir_a['rendah'] & right_ir_b['rendah'] & mid_ir_a['sedang'] & mid_ir_b['sedang'] & mid_ir_c['sedang'] & left_ir_a['tinggi'] & left_ir_b['tinggi'], [rms['sedang'], lms['lambat']])

        rule4 = ctl.Rule(right_ir_a['rendah'] & right_ir_b['sedang'] & mid_ir_a['sedang'] & mid_ir_b['sedang'] & mid_ir_c['sedang'] & left_ir_a['sedang'] & left_ir_b['rendah'], [rms['sedang'], lms['sedang']])

        rule5 = ctl.Rule(right_ir_a['tinggi'] & right_ir_b['tinggi'] & mid_ir_a['sedang'] & mid_ir_b['sedang'] & mid_ir_c['sedang'] & left_ir_a['rendah'] & left_ir_b['rendah'], [rms['lambat'], lms['sedang']])

        rule6 = ctl.Rule(right_ir_a['rendah'] & right_ir_b['sedang'] & mid_ir_a['tinggi'] & mid_ir_b['tinggi'] & mid_ir_c['tinggi'] & left_ir_a['sedang'] & left_ir_b['rendah'], [rms['sedang'], lms['sedang']])

        rule7 = ctl.Rule(right_ir_a['sedang'] & right_ir_b['sedang'] & mid_ir_a['rendah'] & mid_ir_b['rendah'] & mid_ir_c['rendah'] & left_ir_a['sedang'] & left_ir_b['sedang'], [rms['cepat'], lms['lambat']])

        rule8 = ctl.Rule(right_ir_a['sedang'] & right_ir_b['sedang'] & mid_ir_a['tinggi'] & mid_ir_b['tinggi'] & mid_ir_c['tinggi'] & left_ir_a['sedang'] & left_ir_b['sedang'], [rms['lambat'], lms['cepat']])

        # Sistem kontrol fuzzy
        fuzzy_control = ctl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8])
                
        fuzzy_control.input['Right a'] = right_ir_a_value
        fuzzy_control.input['Right b'] = right_ir_b_value
        fuzzy_control.input['Mid a'] = mid_ir_a_value
        fuzzy_control.input['Mid b'] = mid_ir_b_value
        fuzzy_control.input['Mid c'] = mid_ir_c_value
        fuzzy_control.input['Left a'] = left_ir_a_value
        fuzzy_control.input['Left b'] = left_ir_b_value

        # Lakukan perhitungan inferensi
        fuzzy_control.compute()
        
        motor_right_value = fuzzy_control.output['Kecepatan Motor Kanan']
        motor_left_value = fuzzy_control.output['Kecepatan Motor Kiri']

        # Apply motor speeds
        lms.setVelocity(motor_left_value)
        rms.setVelocity(motor_right_value)

        # Tampilkan hasil keluaran
        print("Kecepatan Motor Kanan:", motor_right_value)
        print("Kecepatan Motor Kiri:", motor_left_value)

        rms.view(fuzzy_control)
        lms.view(fuzzy_control)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)