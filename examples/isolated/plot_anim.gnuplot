set datafile separator ";"
set terminal gif animate delay 100 size 800,800
set output "examples/isolated/output.gif"
unset key

# Find step range
stats 'examples/isolated/output.txt' using 1 nooutput
first_step = STATS_min
last_step = STATS_max

# Find x/y range from cell section (not vehicle section!)
stats '< grep ";cell" examples/isolated/output.txt' using 2 nooutput
min_x = STATS_min
max_x = STATS_max
stats '< grep ";cell" examples/isolated/output.txt' using 3 nooutput
min_y = STATS_min
max_y = STATS_max

set xrange [min_x-1:max_x+1]
set yrange [min_y-1:max_y+1]
set xtics 1
set ytics 1
set grid

set palette defined ( 0 "red", 1 "blue", 2 "green", 3 "orange", 4 "purple", 5 "cyan", 6 "magenta", 7 "brown", 8 "black", 9 "gray" )
set cbrange [0:9]
unset colorbox

do for [i=first_step:last_step] {
    set title sprintf("Step: %d", i)
    plot \
        '< grep ";cell" examples/isolated/output.txt' using 2:3 with points pt 6 ps 3 lc rgb "gray" notitle, \
        'examples/isolated/output.txt' using ($1==i ? $8 : 1/0):($1==i ? $9 : 1/0):(int($2)/10) with points pt 7 ps 3 lc palette notitle, \
        'examples/isolated/output.txt' using ($1==i ? $8 : 1/0):($1==i ? $9 : 1/0):(sprintf("id=%d, v=%.0f", $2, $4)) with labels offset 0,1 notitle
}