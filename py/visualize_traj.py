import os
import pathlib
import numpy as np
import matplotlib.pyplot as plt

prj_path = os.getcwd() +'/'
print(prj_path)

filelist = [
 prj_path+"/analysis/prev_to_cur_transformation.txt", #prev
 prj_path+"/analysis/trajectory.txt", #trajectory
 prj_path+"/analysis/smoothed_trajectory.txt" , #smoothe
 prj_path+"/analysis/new_prev_to_cur_transformation.txt", #new_tran
# "/Users/4dreplay/work/dove/prev_to_cur_transformation.txt", #prev
# "/Users/4dreplay/work/dove/trajectory.txt", #trajectory
# "/Users/4dreplay/work/dove/smoothed_trajectory.txt" , #smoothe
# "/Users/4dreplay/work/dove/new_prev_to_cur_transformation.txt" #new_tran
#prj_path+"/analysis/detected_obj.txt",
#prj_path+"/analysis/detected_obj_center.txt"
]

frame_id_o = []
dx_o = []
dy_o = []
da_o = []
frame_id_tra = []
dx_tra = []
dy_tra = []
da_tra = []
frame_id_sm = []
dx_sm = []
dy_sm = []
da_sm = []
frame_id_new = []
dx_new = []
dy_new = []
da_new = []
frame_id_dt = []
ox = []
oy = []
ow = []
oh = []
frame_id_dt_c = []
cx = []
cy = []

for i in filelist :
    f = open(i, 'r')
    lines = f.readlines()
    #print(lines)
    for line in lines :
        text = line.split()
        if i == filelist[0] :
            frame_id_o.append(int(text[0]))
            dx_o.append( float(text[1]))
            dy_o.append( float(text[2]))
            da_o.append( float(text[3]))
        elif i == filelist[1] :
            frame_id_tra.append( int(text[0]))
            dx_tra.append( float(text[1]))
            dy_tra.append( float(text[2]))
            da_tra.append( float(text[3]))
        elif i == filelist[2] :
            frame_id_sm.append( int(text[0]))
            dx_sm.append( float(text[1]))
            dy_sm.append( float(text[2]))
            da_sm.append( float(text[3]))
        elif i == filelist[3] :
            frame_id_new.append( int(text[0]))
            dx_new.append( float(text[1]))
            dy_new.append( float(text[2]))
            da_new.append( float(text[3]))
        elif i == filelist[4] :
            frame_id_dt.append( int(text[0]))
            ox.append( float(text[1]))
            oy.append( float(text[2]))
            #ow.append( float(text[3]))
            #oh.append( float(text[4]))
        elif i == filelist[5] :            
            frame_id_dt_c.append( int(text[0]))            
            cx.append( float(text[1]))
            cy.append( float(text[2]))

print("length : ", len(frame_id_o))
# for a in range(0, len(frame_id_o)) :
#     print('id {} dxo {} dx_tra {} dx_sm {} dx_new {}'.format(frame_id_o[a], dx_o[a], 
#         dx_tra[a], dx_sm[a], dx_new[a]))

#plt.plot(dx_o)
#plt.plot(dy_o)
fig = plt.figure(figsize=(16,8))
ax1 = plt.subplot(2, 3, 1)
ax1.plot(dx_tra)
ax1.plot(dx_sm)
ax1.set_xlabel("frame")
ax1.set_ylabel("accumulated dx/ smoothed dx")

ax2 = plt.subplot(2, 3, 2)
ax2.plot(dy_tra)
ax2.plot(dy_sm)
ax2.set_xlabel("frame")
ax2.set_ylabel("accumulated dy/ smoothed dy")

ax3 = plt.subplot(2, 3, 3)
ax3.plot(dx_o)
ax3.plot(dx_new)
ax3.set_xlabel("frame")
ax3.set_ylabel("dx / new dx")

ax4 = plt.subplot(2, 3, 4)
ax4.plot(dy_o)
ax4.plot(dy_new)
ax4.set_xlabel("frame")
ax4.set_ylabel("dy / new dy")

ax5 = plt.subplot(2, 3, 5)
ax5.plot(ox)
ax5.plot(oy)
ax5.set_xlabel("frame")
ax5.set_ylabel("x / y")

ax6 = plt.subplot(2, 3, 6)
ax6.plot(cx)
ax6.plot(cy)
ax6.set_xlabel("frame")
ax6.set_ylabel(" cx / cy")

plt.show()

# plt.plot(dx_tra)
# plt.plot(dx_sm)
# plt.show()