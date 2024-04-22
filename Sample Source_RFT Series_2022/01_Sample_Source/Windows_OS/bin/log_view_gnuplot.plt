
#the working directory should be changed for gnuplot.
cd 'D:\Projects\Software\VS_Cpp\RT_0000_RFT_SAMPLE\bin'

#modify the data file name
filename = 'RFT_log_20171215_164722.txt'

set multiplot
set size 1.0,0.5
set origin 0,0.5
plot filename using 0:1 t 'Fx' with lines, filename using 0:2 t 'Fy' with lines, filename using 0:3 t 'Fz' with lines,
set origin 0,0
plot filename using 0:4 t 'Tx' with lines, filename using 0:5 t 'Ty' with lines, filename using 0:6 t 'Tz' with lines
unset multiplot
