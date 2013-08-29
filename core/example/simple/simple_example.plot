# gnuplot
set autoscale
set xtic auto
set ytic auto
set title "-a * (du^2)(d^2x) + b * u - c = 0"
set xlabel "Grid Point"
set ylabel "u"
plot "simple_example.dat"