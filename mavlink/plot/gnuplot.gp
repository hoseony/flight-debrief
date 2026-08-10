if (!exists("ARG1")) {
    print "Usage: gnuplot -c plot/gnuplot.gp <csv-file>"
    exit
}

data_file = ARG1

set datafile separator ","
set key autotitle columnhead
set xlabel "Time (seconds)"
set ylabel "Angle (radians)"
set grid

while (1) {
    plot data_file using ($1 / 1000.0):2 with lines title "Roll", \
         data_file using ($1 / 1000.0):3 with lines title "Pitch", \
         data_file using ($1 / 1000.0):4 with lines title "Yaw"

    pause 0.5
}
