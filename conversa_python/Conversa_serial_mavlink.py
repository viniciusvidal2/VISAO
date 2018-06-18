###################################################################################################
####################### Serial Conexion via MAVLink Protocol with APM #############################
###################################################################################################

################################
## Programming language: Python
################################

# --------------------------------------
# Importing some libraries
# --------------------------------------

import sys, struct, time, os
import numpy as np, scipy.io as io
import math
from time import sleep

# --------------------------------------
# Importing MAVLink libray conexion
# --------------------------------------
from pymavlink import mavutil
from pymavlink import DFReader
from pymavlink.dialects.v10 import ardupilotmega as mavlink1

# --------------------------------------
# Initializing MAVLink conexion - Master Object
# --------------------------------------
device = '/dev/ttyACM0'
baudrate = 115200
master = mavutil.mavlink_connection(device, baud=baudrate)

# Waiting for heartbeat message from the APM board
conexao = master.wait_heartbeat() 

# Requesting message transmissions
master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

# --------------------------------
# Classes and functions for the messages
# --------------------------------
# Buffer classe to write and read, necessary to start the MAVLink dialect
class fifo(object): 
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

# Creating one buffer class
f   = fifo()
mav = mavlink1.MAVLink(f) # Dialeto a ser usado, todas as mensagens e tipos aqui

# Aqui ja testando a mensagemn a ser utilizada para enviar os valores de setpoint roll pitch yaw
# msg = mavlink1.MAVLink_rc_channels_override_message(master.target_system, master.target_component, 4000, 4000, 4000, 0, 0, 0, 0, 0)
# msg.pack(mav)
# buf = msg.get_msgbuf()
# sleep(0.001)
# master.write(buf)
# master.write(buf)
# master.write(buf)
# master.write(buf)
# master.write(buf)

# Cria mensagens a partir do dialeto
# m1 = mavlink1.MAVLink_param_set_message(master.target_system, master.target_component, "FLTMODE5", 7, mavlink1.MAV_PARAM_TYPE_INT32)

# Empacota as mensagens e extrai o buffer a ser enviado
# m1.pack(mav)
# b1 = m1.get_msgbuf()

# Envia pelo canal estabelecido serialmente, objeto MASTER
# sleep(0.1) # Para garantir envio
# master.write(b1)

roll_ch  = []
pitch_ch = []
yaw_ch   = []

roll_mt_P  = []
roll_mt_I  = []
roll_mt_D  = []

pitch_mt_P = []
pitch_mt_I = []
pitch_mt_D = []

yaw_mt_P   = []
yaw_mt_I   = []

X = []
Y = []  ## [ cm ]
Z = []

roll  = []
pitch = []
yaw   = []

lmt_roll_pitch  = []
lmt_yaw         = []

## Trocacao
## --------
if conexao:
    for i in range(5000):
        sleep(0.1)
        # msg_rcv = master.recv_msg()
        msg_rcv_att = master.recv_match(type="ATTITUDE", blocking=True)
        msg_rcv_pos = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        # if msg_rcv:
        if msg_rcv_att and msg_rcv_pos:
            # print(msg_rcv.get_type())
            X.append(msg_rcv_pos.vx)
            Y.append(msg_rcv_pos.vy)
            Z.append(msg_rcv_pos.vz)

            roll.append(msg_rcv_att.roll)
            pitch.append(msg_rcv_att.pitch)
            yaw.append(msg_rcv_att.yaw)

            print("X: %.4f \t Y: %.4f \t Z: %.4f"%(msg_rcv_pos.vx/10000, msg_rcv_pos.vy/10000, msg_rcv_pos.vz/10000))
            #Roll_ch  = float(msg_rcv_in.nav_roll)/1e3
            #Pitch_ch = float(msg_rcv_in.nav_pitch)/1e3
            #Yaw_ch   = float(msg_rcv_in.alt_error)/1e3

            #Lmt_roll_pitch = float(msg_rcv_in.nav_bearing)/1e3
            # Lmt_yaw        = float(msg_rcv_in.wp_dist)*1e2
            # print Lmt_yaw

            #Dt_roll   = float(msg_rcv_dt.vx)/1e3
            #Dt_pitch  = float(msg_rcv_dt.vy)/1e3
            #Dt_yaw    = float(msg_rcv_dt.vz)/1e3

            #Roll_mt_P  = float(msg_rcv_out.chan1_scaled)/1e3
            #Roll_mt_I  = float(msg_rcv_out.chan2_scaled)/1e3
            #Roll_mt_D  = float(msg_rcv_out.chan3_scaled)/1e3

            #Pitch_mt_P  = float(msg_rcv_out.chan4_scaled)/1e3
            #Pitch_mt_I  = float(msg_rcv_out.chan5_scaled)/1e3
            #Pitch_mt_D  = float(msg_rcv_out.chan6_scaled)/1e3

            #Yaw_mt_P  = float(msg_rcv_out.chan7_scaled)/1e3
            #Yaw_mt_I  = float(msg_rcv_out.chan8_scaled)/1e3                               

            ##print "Roll In: %f Pitch In: %f Yaw In:  %f"%( Roll_ch, Pitch_ch, Yaw_ch)
            ##print "Roll P:  %f Roll I:   %f Roll D:  %f"%( Roll_mt_P, Roll_mt_I, Roll_mt_D)
            ##print "Pitch P: %f Pitch I:  %f Pitch D: %f"%( Pitch_mt_P, Pitch_mt_I, Pitch_mt_D)
            ##print "Yaw P:   %f Yaw   I:  %f"%( Yaw_mt_P, Yaw_mt_I)            
            ##print "Dt Roll: %f Dt Pitch:  %f Dt Yaw: %f"%( Dt_roll, Dt_pitch, Dt_yaw)
            ##print "-----------------------------------------------------------"

            #roll_ch.append(Roll_ch)
            #pitch_ch.append(Pitch_ch)
            #yaw_ch.append(Yaw_ch)

            #lmt_roll_pitch.append(Lmt_roll_pitch)  
            #lmt_yaw.append(Lmt_yaw)

            #roll_mt_P.append(Roll_mt_P)
            #roll_mt_I.append(Roll_mt_I)
            #roll_mt_D.append(Roll_mt_D)

            #pitch_mt_P.append(Pitch_mt_P)
            #pitch_mt_I.append(Pitch_mt_I)
            #pitch_mt_D.append(Pitch_mt_D)

            #yaw_mt_P.append(Yaw_mt_P)
            #yaw_mt_I.append(Yaw_mt_I)

            #dt_roll.append(float(Dt_roll))            
            #dt_pitch.append(float(Dt_pitch))
            #dt_yaw.append(float(Dt_yaw))

        # print "Iteracao: %d"%i

    #io.savemat('Inputs.mat', {'Roll_Channel': roll_ch,'Pitch_Channel': pitch_ch,'Yaw_Channel': yaw_ch})               
    #io.savemat('Outputs.mat', {'Roll_P': roll_mt_P, 'Roll_I': roll_mt_I, 'Roll_D': roll_mt_D,'Pitch_P': pitch_mt_P, 'Pitch_I': pitch_mt_I, 'Pitch_D': pitch_mt_D, 'Yaw_P': yaw_mt_P, 'Yaw_I': yaw_mt_I}) 
    #io.savemat('Time.mat', {'Dt_Roll': dt_roll, 'Dt_Pitch': dt_pitch, 'Dt_Yaw': dt_yaw})            
    #io.savemat('Limits.mat', {'Lmt_roll_pitch': lmt_roll_pitch, 'Lmt_yaw': lmt_yaw})  
    print("FIM!")