#!/usr/bin/gawk
/^i( [0-9]+){5}/ { sum5 += $5 }
#/^e / { print $0 }
END { print sum5 }
