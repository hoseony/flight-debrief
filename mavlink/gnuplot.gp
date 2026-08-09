set terminal x11 noraise
set datafile separator ","

set title "PX4 Attitude"
set xlabel "Time since boot (seconds)"
set ylabel "Angle (degrees)"
set grid

set key autotitle columnhead

plot "out/attitude.csv" using 1:2 with lines linewidth 2, \
     "" using 1:3 with lines linewidth 2, \
     "" using 1:4 with lines linewidth 2

pause 0.2
reread
