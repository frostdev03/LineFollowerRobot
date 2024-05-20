from controller import Robot
import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctl

def run_robot(robot):

    time_step = 32
    max_speed = 6.28

    #motor
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Input
    right_ir_a = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 0')
    right_ir_b = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 1')
    mid_ir_a = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 2')
    mid_ir_b = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 3')
    mid_ir_c = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 4')
    left_ir_a = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 5')
    left_ir_b = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 6')

    # Output
    rms = ctl.Consequent(np.arange(0, 101, 1), 'RMS (%)')
    lms = ctl.Consequent(np.arange(0, 101, 1), 'LMS (%)')

    # Fungsi keanggotaan untuk sensor
    for sensor in [right_ir_a, right_ir_b, mid_ir_a, mid_ir_b, mid_ir_c, left_ir_a, left_ir_b]:
        sensor['putih'] = fuzz.trapmf(sensor.universe, [0, 0, 400, 600])
        sensor['hitam'] = fuzz.trapmf(sensor.universe, [400, 600, 1000, 1000])

    # Fungsi keanggotaan untuk PWM motor
    rms['lambat'] = fuzz.trimf(rms.universe, [0, 0, 50])
    rms['sedang'] = fuzz.trimf(rms.universe, [0, 50, 100])
    rms['cepat'] = fuzz.trimf(rms.universe, [50, 100, 100])

    lms['lambat'] = fuzz.trimf(lms.universe, [0, 0, 50])
    lms['sedang'] = fuzz.trimf(lms.universe, [0, 50, 100])
    lms['cepat'] = fuzz.trimf(lms.universe, [50, 100, 100])
        
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

        # pembacaan ir
        right_ir_a_value = r_ir_a.getValue()
        right_ir_b_value = r_ir_b.getValue()
        mid_ir_a_value = m_ir_a.getValue()
        mid_ir_b_value = m_ir_b.getValue()
        mid_ir_c_value = m_ir_c.getValue()
        left_ir_a_value = l_ir_a.getValue()
        left_ir_b_value = l_ir_b.getValue()

        print('Left a {} Left b {} Mid a {} Mid b {} Mid c {} Right a {} Right b {} '.format(right_ir_a_value, right_ir_b_value, mid_ir_a_value, mid_ir_b_value, mid_ir_c_value, left_ir_a_value, left_ir_b_value))
        
        # Aturan fuzzy
        rule1 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['lambat']])
        rule2 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['lambat'], lms['lambat']])

        # lurus
        rule3 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['cepat']])

        # belok kanan
        rule4 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['lambat'], lms['sedang']])
        rule5 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['lambat'], lms['cepat']])
        rule6 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['lambat'], lms['cepat']])
        rule7 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['lambat'], lms['cepat']])

        # belok kiri 
        rule8 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['lambat']])
        rule9 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])
        rule10 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])
        rule11 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])
        
        rule12 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
        rule13 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['cepat']])
        rule14 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['cepat']])
        rule15 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['cepat'], lms['lambat']])
        rule16 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['cepat']])
        rule17 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['sedang']])
        rule18 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])

        # Sistem kontrol fuzzy
        fuzzy_control = ctl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18])

        sensor_input = ctl.ControlSystemsensor_input(fuzzy_control)
        
        # Buat simulasi dari kontrol sistem
        sensor_input = ctl.ControlSystemsensor_input(fuzzy_control)

        sensor_input.input['Sensor 0'] = right_ir_a_value
        sensor_input.input['Sensor 1'] = right_ir_b_value
        sensor_input.input['Sensor 2'] = mid_ir_a_value
        sensor_input.input['Sensor 3'] = mid_ir_b_value
        sensor_input.input['Sensor 4'] = mid_ir_c_value
        sensor_input.input['Sensor 5'] = left_ir_a_value
        sensor_input.input['Sensor 6'] = left_ir_b_value

        # Lakukan perhitungan inferensi
        sensor_input.compute()
        
        pwm_motor_kanan = sensor_input.output['RMS (%)']
        pwm_motor_kiri = sensor_input.output['LMS (%)']
        
        # Konversi output menjadi kecepatan motor yang sesuai dengan rentang max_speed
        kecepatan_motor_kanan = -max_speed * (pwm_motor_kanan / 100)
        kecepatan_motor_kiri = -max_speed * (pwm_motor_kiri / 100)
        
        # Atur kecepatan motor
        left_motor.setVelocity(kecepatan_motor_kiri)
        right_motor.setVelocity(kecepatan_motor_kanan)
        
        # sensor_input.output['RMS (%)'] = -max_speed
        # sensor_input.output['LMS (%)'] = -max_speed
        
        # Tampilkan hasil keluaran
        print("PWM Motor Kanan:", sensor_input.output['RMS (%)'])
        print("PWM Motor Kiri:", sensor_input.output['LMS (%)'])
    

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)