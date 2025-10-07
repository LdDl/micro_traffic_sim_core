set datafile separator ";"
set terminal gif animate delay 100 size 800,200
set output "examples/nasch-one-lane/output.gif"
unset key

# Find step range
stats 'examples/nasch-one-lane/output.txt' using 1 nooutput
first_step = STATS_min
last_step = STATS_max

# Find x/y range from grid cells only
stats 'examples/nasch-one-lane/output.txt' every ::1::20 using 2 nooutput
min_x = STATS_min
max_x = STATS_max
stats 'examples/nasch-one-lane/output.txt' every ::1::20 using 3 nooutput
min_y = STATS_min
max_y = STATS_max

set xrange [min_x-1:max_x+1]
set yrange [min_y-1:max_y+1]
set xtics 1
set xtics format "%g"

set palette defined ( 0 "red", 1 "blue", 2 "green", 3 "orange", 4 "purple", 5 "cyan", 6 "magenta", 7 "brown", 8 "black", 9 "gray" )
set cbrange [0:9]
unset colorbox

do for [i=first_step:last_step] {
    plot \
        'examples/nasch-one-lane/output.txt' every ::1::20 using 2:3 with points pt 6 ps 3 lc rgb "gray" notitle, \
        'examples/nasch-one-lane/output.txt' every ::21 using ($1<=i && $2==0 ? $8 : 1/0):($1<=i && $2==0 ? $9 : 1/0):($2) with points pt 7 ps 2 lc palette notitle, \
        'examples/nasch-one-lane/output.txt' every ::21 using ($1<i && $2==0 ? $8 : 1/0):($1<i && $2==0 ? $9 : 1/0):("p") with labels offset 0,1 font ",8" notitle, \
        'examples/nasch-one-lane/output.txt' every ::21 using ($1==i ? $8 : 1/0):($1==i ? $9 : 1/0):($2) with points pt 7 ps 3 lc palette notitle, \
        'examples/nasch-one-lane/output.txt' every ::21 using ($1==i ? $8 : 1/0):($1==i ? $9 : 1/0):(sprintf("v=%.0f", $4)) with labels offset 0,1 notitle
}
set output