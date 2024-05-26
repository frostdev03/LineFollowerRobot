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
    rms['sedang'] = fuzz.trimf(rms.universe, [50, 50, 100])
    rms['cepat'] = fuzz.trimf(rms.universe, [100, 100, 100])

    lms['lambat'] = fuzz.trimf(lms.universe, [0, 0, 50])
    lms['sedang'] = fuzz.trimf(lms.universe, [50, 50, 100])
    lms['cepat'] = fuzz.trimf(lms.universe, [100, 100, 100])
        
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
    
    # fuzzy rule
    # lurus
    rule1 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['cepat']])
    rule2 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['cepat'], lms['cepat']])
    rule3 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['cepat']])
    rule4 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['cepat']])
    rule5 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['lambat'], lms['sedang']])
    rule6 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['lambat'], lms['sedang']])
    rule7 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule8 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['cepat'], lms['lambat']])
    rule9 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['cepat'], lms['lambat']])
    rule10 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['cepat'], lms['lambat']])
    rule11 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['cepat']])
    rule12 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['cepat']])
    rule13 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['cepat']])
    rule14 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])
    rule15 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['sedang'], lms['sedang']])
    rule16 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
    rule17 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
    rule18 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
    rule19 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
    rule20 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])
    rule21 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])
    rule22 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['cepat'], lms['lambat']])
    rule23 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])
    rule24 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['cepat'], lms['lambat']])
    rule25 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['sedang']])
    rule26 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['sedang']])
    rule27 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['cepat'], lms['sedang']])
    rule28 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
    rule29 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
    rule30 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['cepat']])
    rule31 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['cepat']])
    rule32 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['cepat'], lms['sedang']])
    rule33 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
    rule34 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule35 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['sedang'], lms['lambat']])
    rule36 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule37 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule38 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule39 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule40 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['sedang'], lms['lambat']])
    rule41 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['sedang'], lms['lambat']])
    rule42 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['lambat'], lms['sedang']])
    rule43 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['lambat'], lms['sedang']])
    rule44 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['lambat'], lms['sedang']])
    rule45 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule46 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['lambat'], lms['sedang']])
    rule47 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule48 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['lambat'], lms['sedang']])
    rule49 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['cepat'], lms['sedang']])
    rule50 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['putih'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['cepat']])
    rule51 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['sedang'], lms['lambat']])
    rule52 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['cepat'], lms['lambat']])
    rule53 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['hitam'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['cepat'], lms['sedang']])
    rule54 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['lambat'], lms['sedang']])
    rule55 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['putih'] & left_ir_b['hitam'], [rms['sedang'], lms['sedang']])
    rule56 = ctl.Rule(right_ir_a['putih'] & right_ir_b['putih'] & mid_ir_a['putih'] & mid_ir_b['hitam'] & mid_ir_c['hitam'] & left_ir_a['putih'] & left_ir_b['putih'], [rms['sedang'], lms['lambat']])
    rule57 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['sedang'], lms['sedang']])
    rule58 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['hitam'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['putih'], [rms['lambat'], lms['sedang']])
    rule59 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule60 = ctl.Rule(right_ir_a['hitam'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['putih'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['sedang']])
    rule61 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
    rule62 = ctl.Rule(right_ir_a['putih'] & right_ir_b['hitam'] & mid_ir_a['putih'] & mid_ir_b['putih'] & mid_ir_c['hitam'] & left_ir_a['hitam'] & left_ir_b['hitam'], [rms['sedang'], lms['lambat']])
            
    # Sistem kontrol fuzzy
    fuzzy_control = ctl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20, rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30, rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40, rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49, rule50, rule51, rule52, rule53, rule54, rule55, rule56, rule57, rule58, rule59, rule60, rule61, rule62])
    
    sensor_input = ctl.ControlSystemSimulation(fuzzy_control)

    while robot.step(time_step) != -1:

        # pembacaan ir
        right_ir_a_value = r_ir_a.getValue()
        right_ir_b_value = r_ir_b.getValue()
        mid_ir_a_value = m_ir_a.getValue()
        mid_ir_b_value = m_ir_b.getValue()
        mid_ir_c_value = m_ir_c.getValue()
        left_ir_a_value = l_ir_a.getValue()
        left_ir_b_value = l_ir_b.getValue()

        print('Right a {} Right b {} Mid a {} Mid b {} Mid c {} Left a {} Left b {} '.format(right_ir_a_value, right_ir_b_value, mid_ir_a_value, mid_ir_b_value, mid_ir_c_value, left_ir_a_value, left_ir_b_value))

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
        right_motor.setVelocity(kecepatan_motor_kanan)
        left_motor.setVelocity(kecepatan_motor_kiri)
        
        # Tampilkan hasil keluaran
        print("PWM Motor Kanan:", kecepatan_motor_kanan)
        print("PWM Motor Kiri:", kecepatan_motor_kiri)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)