#!/usr/bin/gawk
/^z( [0-9]+){3}/ { cnt_p += $2; cnt_r += $3; cnt_v += $4; cnt += 1 }
END { print "#"cnt; print "P"cnt_p; print "R"cnt_r; print "V"cnt_v }
