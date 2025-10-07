set datafile separator ";"
set terminal gif animate delay 100 size 800,200
set output "examples/nasch-two-lanes/output.gif"
set xrange [-2:20]
set yrange [0:4]
set xtics 1
set xtics format "%g"
unset key

set palette defined ( 0 "red", 1 "blue", 2 "green", 3 "orange", 4 "purple", 5 "cyan", 6 "magenta", 7 "brown", 8 "black", 9 "gray" )
set cbrange [0:9]
unset colorbox

do for [i=-1:17] {
    plot \
        'examples/nasch-two-lanes/output.txt' every ::1::40 using 2:3 with points pt 6 ps 3 lc rgb "gray" notitle, \
        'examples/nasch-two-lanes/output.txt' every ::42 using ($1<=i && $2==0 ? $8 : 1/0):($1<=i && $2==0 ? $9 : 1/0):($2) with points pt 7 ps 2 lc palette notitle, \
        'examples/nasch-two-lanes/output.txt' every ::42 using ($1<=i && $2==1 ? $8 : 1/0):($1<=i && $2==1 ? $9 : 1/0):($2) with points pt 7 ps 2 lc palette notitle, \
        'examples/nasch-two-lanes/output.txt' every ::42 using ($1<i && $2==0 ? $8 : 1/0):($1<i && $2==0 ? $9 : 1/0):("p") with labels offset 0,1 font ",8" notitle, \
        'examples/nasch-two-lanes/output.txt' every ::42 using ($1<i && $2==1 ? $8 : 1/0):($1<=i && $2==1 ? $9 : 1/0):("p") with labels offset 0,1 font ",8" notitle, \
        'examples/nasch-two-lanes/output.txt' every ::42 using ($1==i ? $8 : 1/0):($1==i ? $9 : 1/0):($2) with points pt 7 ps 3 lc palette notitle, \
        'examples/nasch-two-lanes/output.txt' every ::42 using ($1==i ? $8 : 1/0):($1==i ? $9 : 1/0):(sprintf("v=%.0f", $4)) with labels offset 0,1 notitle
}
set output