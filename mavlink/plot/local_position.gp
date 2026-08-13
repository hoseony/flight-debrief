set datafile separator ","
set xlabel "East (m)"
set ylabel "North (m)"
set zlabel "Up (m)"
set view equal xyz

splot "<your-path>/local_position.csv" \
    using 3:2:(-$4) with lines title "Flight path"
