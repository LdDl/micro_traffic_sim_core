set datafile separator ";"
set terminal gif animate delay 100 size 800,200
set output "examples/nasch-lanes-merge/output.gif"
unset key

# Get grid cell x/y range (lines 2–16)
stats 'examples/nasch-lanes-merge/output.txt' every ::1::15 using 2 nooutput
min_x = STATS_min
max_x = STATS_max
stats 'examples/nasch-lanes-merge/output.txt' every ::1::15 using 3 nooutput
min_y = STATS_min
max_y = STATS_max

set xrange [min_x-1:max_x+1]
set yrange [min_y-1:max_y+1]
set xtics 1
set xtics format "%g"

set palette defined ( 0 "red", 1 "blue", 2 "green", 3 "orange", 4 "purple", 5 "cyan", 6 "magenta", 7 "brown", 8 "black", 9 "gray" )
set cbrange [0:9]
unset colorbox

# Get step range from vehicle data (lines 18+)
stats 'examples/nasch-lanes-merge/output.txt' every ::17 using 1 nooutput
first_step = STATS_min
last_step = STATS_max

do for [i=-1:8] {
    plot \
        'examples/nasch-lanes-merge/output.txt' every ::1::15 using 2:3 with points pt 6 ps 3 lc rgb "gray" notitle, \
        'examples/nasch-lanes-merge/output.txt' every ::17 using ($1<=i && $2==0 ? $8 : 1/0):($1<=i && $2==0 ? $9 : 1/0):($2) with points pt 7 ps 2 lc palette notitle, \
        'examples/nasch-lanes-merge/output.txt' every ::17 using ($1<=i && $2==1 ? $8 : 1/0):($1<=i && $2==1 ? $9 : 1/0):($2) with points pt 7 ps 2 lc palette notitle, \
        'examples/nasch-lanes-merge/output.txt' every ::17 using ($1<i && $2==0 ? $8 : 1/0):($1<i && $2==0 ? $9 : 1/0):("p") with labels offset 0,1 font ",8" notitle, \
        'examples/nasch-lanes-merge/output.txt' every ::17 using ($1<i && $2==1 ? $8 : 1/0):($1<i && $2==1 ? $9 : 1/0):("p") with labels offset 0,1 font ",8" notitle, \
        'examples/nasch-lanes-merge/output.txt' every ::17 using ($1==i ? $8 : 1/0):($1==i ? $9 : 1/0):($2) with points pt 7 ps 3 lc palette notitle, \
        'examples/nasch-lanes-merge/output.txt' every ::17 using ($1==i ? $8 : 1/0):($1==i ? $9 : 1/0):(sprintf("v=%.0f", $4)) with labels offset 0,1 notitle
}
set output