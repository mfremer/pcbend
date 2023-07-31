#!/usr/bin/gawk
/^i( [0-9]+){5}/ { sum4 += $4 }
END { print sum4 }
