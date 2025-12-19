# Animated GIF visualization for all-tail simulation
# Shows multiple vehicles on isolated sub-networks

# Get directory from command line or use default
if (!exists("dir")) dir = ""

# Load auto-generated config
load dir."config.gnu"

# Output settings
set terminal gif animate delay 50 size 1200,800
set output dir."simulation.gif"

# Plot settings
set xrange [0:22]
set yrange [-17:10]
set xlabel "X"
set ylabel "Y"
set grid

# Style for road cells
set style line 1 lc rgb '#CCCCCC' pt 7 ps 1.5

# Styles for vehicle heads (different colors per vehicle)
set style line 10 lc rgb '#FF0000' pt 7 ps 2.5  # Vehicle 1 head - Red
set style line 11 lc rgb '#00AA00' pt 7 ps 2.5  # Vehicle 2 head - Green
set style line 12 lc rgb '#0066FF' pt 7 ps 2.5  # Vehicle 3 head - Blue
set style line 13 lc rgb '#FF8800' pt 7 ps 2.5  # Vehicle 4 head - Orange
set style line 14 lc rgb '#FF00FF' pt 7 ps 2.5  # Vehicle 5 head - Bright Magenta
set style line 15 lc rgb '#00FFFF' pt 7 ps 2.5  # Vehicle 6 head - Cyan
set style line 16 lc rgb '#FFCC00' pt 7 ps 2.5  # Vehicle 7 head - Gold
set style line 17 lc rgb '#CC00CC' pt 7 ps 2.5  # Vehicle 8 head - Purple-Pink
set style line 18 lc rgb '#00CC66' pt 7 ps 2.5  # Vehicle 9 head - Sea Green
set style line 19 lc rgb '#FF6666' pt 7 ps 2.5  # Vehicle 10 head - Salmon
set style line 30 lc rgb '#6666FF' pt 7 ps 2.5  # Vehicle 11 head - Light Blue
set style line 31 lc rgb '#66FF66' pt 7 ps 2.5  # Vehicle 12 head - Light Green
set style line 32 lc rgb '#FF66CC' pt 7 ps 2.5  # Vehicle 13 head - Pink
set style line 33 lc rgb '#99FF00' pt 7 ps 2.5  # Vehicle 14 head - Lime (trip-generated)
set style line 34 lc rgb '#FF9966' pt 7 ps 2.5  # Vehicle 15 head - Coral
set style line 35 lc rgb '#66CCCC' pt 7 ps 2.5  # Vehicle 16 head - Light Teal
set style line 36 lc rgb '#CC99FF' pt 7 ps 2.5  # Vehicle 17 head - Light Purple

# Styles for vehicle tails (distinct contrasting colors)
set style line 20 lc rgb '#990066' pt 7 ps 2    # Vehicle 1 tail - Dark Magenta
set style line 21 lc rgb '#006666' pt 7 ps 2    # Vehicle 2 tail - Teal
set style line 22 lc rgb '#9900FF' pt 7 ps 2    # Vehicle 3 tail - Purple
set style line 23 lc rgb '#996600' pt 7 ps 2    # Vehicle 4 tail - Brown
set style line 24 lc rgb '#AA0066' pt 7 ps 2    # Vehicle 5 tail - Dark Pink
set style line 25 lc rgb '#006699' pt 7 ps 2    # Vehicle 6 tail - Dark Cyan
set style line 26 lc rgb '#997700' pt 7 ps 2    # Vehicle 7 tail - Dark Gold
set style line 27 lc rgb '#770077' pt 7 ps 2    # Vehicle 8 tail - Dark Purple
set style line 28 lc rgb '#007744' pt 7 ps 2    # Vehicle 9 tail - Dark Sea Green
set style line 29 lc rgb '#993333' pt 7 ps 2    # Vehicle 10 tail - Dark Red
set style line 40 lc rgb '#333399' pt 7 ps 2    # Vehicle 11 tail - Dark Blue
set style line 41 lc rgb '#339933' pt 7 ps 2    # Vehicle 12 tail - Dark Green
set style line 42 lc rgb '#993366' pt 7 ps 2    # Vehicle 13 tail - Dark Pink
set style line 43 lc rgb '#669900' pt 7 ps 2    # Vehicle 14 tail - Dark Lime
set style line 44 lc rgb '#994433' pt 7 ps 2    # Vehicle 15 tail - Dark Coral
set style line 45 lc rgb '#336666' pt 7 ps 2    # Vehicle 16 tail - Dark Teal
set style line 46 lc rgb '#663399' pt 7 ps 2    # Vehicle 17 tail - Dark Purple

# Create animation frames
do for [i=0:num_steps-1] {
    filename = sprintf(dir."vehicle_step_%02d.dat", i)

    set title sprintf("All Tail Tests - Step %d", i)

    plot dir.'cells.dat' using 2:3 with points ls 1 notitle, \
         dir.'edges.dat' using 1:2:3:4:(strcol(5) eq 'F' ? 0x00AA00 : 0xFF8800) with vectors filled head lw 1 lc rgb variable notitle, \
         filename using ($3==1 && strcol(4) eq 'head' ? $1 : 1/0):($3==1 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 10 title "V1", \
         filename using ($3==1 && strcol(4) eq 'tail' ? $1 : 1/0):($3==1 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 20 notitle, \
         filename using ($3==2 && strcol(4) eq 'head' ? $1 : 1/0):($3==2 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 11 title "V2", \
         filename using ($3==2 && strcol(4) eq 'tail' ? $1 : 1/0):($3==2 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 21 notitle, \
         filename using ($3==3 && strcol(4) eq 'head' ? $1 : 1/0):($3==3 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 12 title "V3", \
         filename using ($3==3 && strcol(4) eq 'tail' ? $1 : 1/0):($3==3 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 22 notitle, \
         filename using ($3==4 && strcol(4) eq 'head' ? $1 : 1/0):($3==4 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 13 title "V4", \
         filename using ($3==4 && strcol(4) eq 'tail' ? $1 : 1/0):($3==4 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 23 notitle, \
         filename using ($3==5 && strcol(4) eq 'head' ? $1 : 1/0):($3==5 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 14 title "V5", \
         filename using ($3==5 && strcol(4) eq 'tail' ? $1 : 1/0):($3==5 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 24 notitle, \
         filename using ($3==6 && strcol(4) eq 'head' ? $1 : 1/0):($3==6 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 15 title "V6", \
         filename using ($3==6 && strcol(4) eq 'tail' ? $1 : 1/0):($3==6 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 25 notitle, \
         filename using ($3==7 && strcol(4) eq 'head' ? $1 : 1/0):($3==7 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 16 title "V7", \
         filename using ($3==7 && strcol(4) eq 'tail' ? $1 : 1/0):($3==7 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 26 notitle, \
         filename using ($3==8 && strcol(4) eq 'head' ? $1 : 1/0):($3==8 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 17 title "V8", \
         filename using ($3==8 && strcol(4) eq 'tail' ? $1 : 1/0):($3==8 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 27 notitle, \
         filename using ($3==9 && strcol(4) eq 'head' ? $1 : 1/0):($3==9 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 18 title "V9", \
         filename using ($3==9 && strcol(4) eq 'tail' ? $1 : 1/0):($3==9 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 28 notitle, \
         filename using ($3==10 && strcol(4) eq 'head' ? $1 : 1/0):($3==10 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 19 title "V10", \
         filename using ($3==10 && strcol(4) eq 'tail' ? $1 : 1/0):($3==10 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 29 notitle, \
         filename using ($3==11 && strcol(4) eq 'head' ? $1 : 1/0):($3==11 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 30 title "V11", \
         filename using ($3==11 && strcol(4) eq 'tail' ? $1 : 1/0):($3==11 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 40 notitle, \
         filename using ($3==12 && strcol(4) eq 'head' ? $1 : 1/0):($3==12 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 31 title "V12", \
         filename using ($3==12 && strcol(4) eq 'tail' ? $1 : 1/0):($3==12 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 41 notitle, \
         filename using ($3==13 && strcol(4) eq 'head' ? $1 : 1/0):($3==13 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 32 title "V13", \
         filename using ($3==13 && strcol(4) eq 'tail' ? $1 : 1/0):($3==13 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 42 notitle, \
         filename using ($3==14 && strcol(4) eq 'head' ? $1 : 1/0):($3==14 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 33 title "V14", \
         filename using ($3==14 && strcol(4) eq 'tail' ? $1 : 1/0):($3==14 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 43 notitle, \
         filename using ($3==15 && strcol(4) eq 'head' ? $1 : 1/0):($3==15 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 34 title "V15", \
         filename using ($3==15 && strcol(4) eq 'tail' ? $1 : 1/0):($3==15 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 44 notitle, \
         filename using ($3==16 && strcol(4) eq 'head' ? $1 : 1/0):($3==16 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 35 title "V16", \
         filename using ($3==16 && strcol(4) eq 'tail' ? $1 : 1/0):($3==16 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 45 notitle, \
         filename using ($3==17 && strcol(4) eq 'head' ? $1 : 1/0):($3==17 && strcol(4) eq 'head' ? $2 : 1/0) with points ls 36 title "V17", \
         filename using ($3==17 && strcol(4) eq 'tail' ? $1 : 1/0):($3==17 && strcol(4) eq 'tail' ? $2 : 1/0) with points ls 46 notitle, \
         filename using (strcol(4) eq 'head' ? $1 : 1/0):(strcol(4) eq 'head' ? $2 : 1/0):(sprintf("V%d", $3)) with labels font ',8' offset 0,1 notitle
}

set output
print "Animation saved to simulation.gif"
