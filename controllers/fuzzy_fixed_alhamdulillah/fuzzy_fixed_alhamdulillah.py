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
    kanan_tajam = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 0')
    kanan = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 1')
    tengah_kanan = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 2')
    tengah = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 3')
    tengah_kiri = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 4')
    kiri = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 5')
    kiri_tajam = ctl.Antecedent(np.arange(0, 1001, 1), 'Sensor 6')

    # Output
    rms = ctl.Consequent(np.arange(0, 101, 1), 'RMS (%)')
    lms = ctl.Consequent(np.arange(0, 101, 1), 'LMS (%)')

    # Fungsi keanggotaan untuk sensor
    for sensor in [kanan_tajam, kanan, tengah_kanan, tengah, tengah_kiri, kiri, kiri_tajam]:
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
    kanan_tajam_val = robot.getDevice('ir0')
    kanan_tajam_val.enable(time_step)

    kanan_val = robot.getDevice('ir1')
    kanan_val.enable(time_step)
    
    tengah_kanan_val = robot.getDevice('ir2')
    tengah_kanan_val.enable(time_step)
    
    tengah_val = robot.getDevice('ir3')
    tengah_val.enable(time_step)
    
    tengah_kiri_val = robot.getDevice('ir4')
    tengah_kiri_val.enable(time_step)
    
    kiri_val = robot.getDevice('ir5')
    kiri_val.enable(time_step)
    
    kiri_tajam_val = robot.getDevice('ir6')
    kiri_tajam_val.enable(time_step)
    
    # fuzzy rule
    rule1 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['cepat']])
    rule2 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['cepat'], lms['cepat']])
    rule3 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['cepat']])
    rule4 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['cepat']])
    rule5 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['cepat']])

    # belok kanan
    rule6 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['lambat'], lms['sedang']])
    rule7 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['putih'], [rms['lambat'], lms['sedang']])
    rule8 = ctl.Rule(kanan_tajam['putih'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['lambat'], lms['sedang']])
    rule9 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['lambat'], lms['cepat']])
    rule10 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['lambat'], lms['cepat']])
    rule11 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['hitam'], [rms['lambat'], lms['cepat']])
    rule12 = ctl.Rule(kanan_tajam['hitam'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['lambat'], lms['cepat']])
    rule13 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['lambat'], lms['cepat']])
    rule14 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['putih'] & kiri_tajam['putih'], [rms['lambat'], lms['cepat']])
    rule15 = ctl.Rule(kanan_tajam['putih'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['putih'], [rms['sedang'], lms['sedang']])
    rule16 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['putih'], [rms['sedang'], lms['cepat']])
    rule17 = ctl.Rule(kanan_tajam['putih'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['putih'] & kiri_tajam['putih'], [rms['sedang'], lms['cepat']])
    rule18 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['sedang'], lms['cepat']])
    rule19 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['hitam'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['sedang'], lms['cepat']])

    # belok kiri 
    rule20 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['sedang'], lms['lambat']])
    rule21 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['hitam'], [rms['sedang'], lms['lambat']])
    rule22 = ctl.Rule(kanan_tajam['hitam'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['sedang'], lms['lambat']])
    rule23 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['lambat']])
    rule24 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['lambat']])
    rule25 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['putih'], [rms['cepat'], lms['lambat']])
    rule26 = ctl.Rule(kanan_tajam['putih'] & kanan['hitam'] & tengah_kanan['hitam'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['cepat'], lms['lambat']])
    rule27 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['cepat'], lms['lambat']])
    rule28 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['cepat'], lms['lambat']])
    rule29 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['cepat'], lms['lambat']])
    rule30 = ctl.Rule(kanan_tajam['hitam'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['hitam'], [rms['sedang'], lms['sedang']])
    rule31 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['hitam'], [rms['cepat'], lms['sedang']])
    rule32 = ctl.Rule(kanan_tajam['hitam'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['putih'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['cepat'], lms['sedang']])
    rule33 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['cepat'], lms['sedang']])
    rule34 = ctl.Rule(kanan_tajam['hitam'] & kanan['hitam'] & tengah_kanan['putih'] & tengah['hitam'] & tengah_kiri['hitam'] & kiri['hitam'] & kiri_tajam['hitam'], [rms['cepat'], lms['sedang']])
    
    rule35 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['hitam'] & tengah_kiri['putih'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['cepat']])
    rule36 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['hitam'] & tengah['putih'] & tengah_kiri['hitam'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['cepat']])
    rule37 = ctl.Rule(kanan_tajam['putih'] & kanan['putih'] & tengah_kanan['putih'] & tengah['putih'] & tengah_kiri['hitam'] & kiri['putih'] & kiri_tajam['putih'], [rms['cepat'], lms['sedang']])

    # Sistem kontrol fuzzy
    fuzzy_control = ctl.ControlSystem([
        rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, 
        rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, 
        rule20, rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, 
        rule29, rule30, rule31, rule32, rule33, rule34, rule35, rule36, rule37,
        ])

    while robot.step(time_step) != -1:

        # pembacaan ir
        kanan_tajam_value = kanan_tajam_val.getValue()
        kanan_value = kanan_val.getValue()
        tengah_kanan_value = tengah_kanan_val.getValue()
        tengah_value = tengah_val.getValue()
        tengah_kiri_value = tengah_kiri_val.getValue()
        kiri_value = kiri_val.getValue()
        kiri_tajam_value = kiri_tajam_val.getValue()

        print('Right a {} Right b {} Mid a {} Mid b {} Mid c {} Left a {} Left b {} '.format(kanan_tajam_value, kanan_value, tengah_kanan_value, tengah_value, tengah_kiri_value, kiri_value, kiri_tajam_value))
        
        # Buat simulasi dari kontrol sistem
        sensor_input = ctl.ControlSystemSimulation(fuzzy_control)

        sensor_input.input['Sensor 0'] = kanan_tajam_value
        sensor_input.input['Sensor 1'] = kanan_value
        sensor_input.input['Sensor 2'] = tengah_kanan_value
        sensor_input.input['Sensor 3'] = tengah_value
        sensor_input.input['Sensor 4'] = tengah_kiri_value
        sensor_input.input['Sensor 5'] = kiri_value
        sensor_input.input['Sensor 6'] = kiri_tajam_value

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
        
        # Atur kecepatan motor
        # right_motor.setVelocity(pwm_motor_kanan)
        # left_motor.setVelocity(pwm_motor_kiri)
        
        # Tampilkan hasil keluaran
        print("PWM Motor Kanan:", kecepatan_motor_kanan)
        print("PWM Motor Kiri:", kecepatan_motor_kiri)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)