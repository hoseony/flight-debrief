if (!exists("ARG1")) {
    print "Usage: gnuplot -c plot/local_position.gp <csv-file>"
    exit
}

data_file = ARG1

set datafile separator ","
set xlabel "East (m)"
set ylabel "North (m)"
set zlabel "Up (m)"
set title "Local Position"
set grid
set view equal xyz

splot data_file using 3:2:(-$4) with lines title "Flight path"

pause mouse close
