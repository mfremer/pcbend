#!/usr/bin/gawk
# float_regex -> [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?

BEGIN {
  RS = "\ne [0-9]+\n" ; FS = " " ;
  
  min_init_val = 1e15;
  max_init_val = 0;
  
  max_id = max_init_val;
  num_faces = 0;
  num_modules = 0;
  num_connectors = 0;

  # Placer vars
  time_threshold_ms_placer = 6.0e4; 
  # Router vars
  time_threshold_ms_router = 8000;
}
/t [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/ { 
  match(RT, /[0-9]+/, vals); id = strtonum(vals[0]);
  if (id > max_id) { max_id = id; num_faces = max_id + 1; }
  
  match($0, /t [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/, mtch); # get only matched substring
  patsplit(mtch[0], vals, /[+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/) # triangle vertex coordinates
  
  x0 = vals[1]; y0 = vals[2];
  x1 = vals[3]; y1 = vals[4];
  x2 = vals[5]; y2 = vals[6];
  triangle_area[id] = 0.5 * ((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0));
}
/n [0-1] [0-1] [0-1]/ {
  match(RT, /[0-9]+/, vals); id = strtonum(vals[0]);
  if (id > max_id) { max_id = id; num_faces = max_id + 1; }
  
  match($0, /n [0-1] [0-1] [0-1]/, mtch); # get only matched substring
  patsplit(mtch[0], vals, /[0-1]/) # triangle vertex coordinates
  
  num_hinges_all[id] += vals[1] + vals[2] + vals[3];
}
/h [0-1] [0-1] [0-1]/ {
  match(RT, /[0-9]+/, vals); id = strtonum(vals[0]);
  if (id > max_id) { max_id = id; num_faces = max_id + 1; }
  
  match($0, /h [0-1] [0-1] [0-1]/, mtch); # get only matched substring
  patsplit(mtch[0], vals, /[0-1]/) # triangle vertex coordinates
  
  num_hinges_half[id] += vals[1] + vals[2] + vals[3];
}
/m [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/ {
  patsplit($0, vals, /m [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/); # all module records in vals
  num_modules += length(vals);
  
  for (i = 1; i <= length(vals); ++i) {
    match(vals[i], /m [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/, mtch); # get only matched substring
    patsplit(mtch[0], vtcs, /[+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/) # module vertex coordinates
    
    x0 = vtcs[1]; y0 = vtcs[2];
    x1 = vtcs[3]; y1 = vtcs[4];
    x2 = vtcs[5]; y2 = vtcs[6];
    area = sqrt(((x1 - x0)^2 + (y1 - y0)^2) * ((x2 - x0)^2 + (y2 - y0)^2));
    module_area[id][i] = area;
  }
}
/p [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/ {
  patsplit($0, vals, /p [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/); # all module records in vals
  num_connectors += length(vals);
  
  for (i = 1; i <= length(vals); ++i) {
    match(vals[i], /p [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/, mtch); # get only matched substring
    patsplit(mtch[0], vtcs, /[+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)?/) # module vertex coordinates
    
    x0 = vtcs[1]; y0 = vtcs[2];
    x1 = vtcs[3]; y1 = vtcs[4];
    x2 = vtcs[5]; y2 = vtcs[6];
    area = sqrt(((x1 - x0)^2 + (y1 - y0)^2) * ((x2 - x0)^2 + (y2 - y0)^2));
    connector_area[id][i] = area;
  }
}
# Pipeline summary info
/z [0-9]+ [0-9]+ [0-9]+/ {
  match(RT, /[0-9]+/, vals); id = strtonum(vals[0]); # get FID
  match($0, /z [0-9]+ [0-9]+ [0-9]+/, mtch); # get only matched substring
  patsplit(mtch[0], vals, /[0-9]+/); # get fields matched by regex

  face_ops[id][1] = vals[1]; # placer
  face_ops[id][2] = vals[2]; # router
  face_ops[id][3] = vals[3]; # verifier
}
# Placer records
/P [0-9]+ [0-9]+/ {
  match(RT, /[0-9]+/, vals); id = strtonum(vals[0]); # get FID
  patsplit($0, vals, /P [0-9]+ [0-9]+/); # all placer records in vals
  if (length(vals) != face_ops[id][1]) { print "["id"]\tPlacer iterations mismatch"; exit; }
  
  for (i in vals) { # for every placer record
    split(vals[i], fields, " "); # get placer details in fields
    
    placer_records[id][i][1] = fields[2]; # leds
    placer_records[id][i][2] = fields[3]; # time
    
    is_bad = (fields[3] > time_threshold_ms_placer);
    if (!(id in bad_placer)) { bad_placer[id] = is_bad; }
    else { bad_placer[id] = bad_placer[id] || is_bad; }
    #print vals[i], " -> "fields[3]" => "is_bad", "bad_placer[id];
  }
}
# Router records
/R [[:alpha:]] [0-9]+ [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?[0-9]+ [0-9]+/ {
  match(RT, /[0-9]+/, vals); id = strtonum(vals[0]);
  
  patsplit($0, vals, /R [[:alpha:]] [0-9]+ [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?([0-9]*[.])?[0-9]+(e[+-]?[0-9]+)? [+-]?[0-9]+ [0-9]+/);
  if (length(vals) != face_ops[id][2]) { print "["id"]\tRouter iterations mismatch"; exit; }
  
  for (i in vals) {
    split(vals[i], fields, " ");

# PREVIOUS ROUTER RECORDS
#    router_records[id][i][1] = fields[2]; # exit status
#    router_records[id][i][2] = fields[3]; # num inter
#    router_records[id][i][3] = fields[4]; # length
#    router_records[id][i][4] = fields[5]; # spacing
#    router_records[id][i][5] = fields[6]; # min_max_spacing 0 inters
#    router_records[id][i][6] = fields[7]; # min_max_spacing passing
#    router_records[id][i][7] = fields[8]; # time

# NEW ROUTER RECORDS
    router_records[id][i][1] = fields[2]; # exit status
    router_records[id][i][2] = fields[3]; # num inter
    router_records[id][i][3] = fields[4]; # length
    router_records[id][i][4] = fields[5]; # io_spacing
    router_records[id][i][5] = fields[6]; # spacing
    router_records[id][i][6] = fields[7]; # best_io_spacing
    router_records[id][i][7] = fields[8]; # max_spacing 0 inters
    router_records[id][i][8] = fields[9]; # longest_unst_subseq
    router_records[id][i][9] = fields[10]; # time

    is_bad = (fields[11] > time_threshold_ms_router);
    if (!(id in bad_router)) { bad_router[id] = is_bad; }
    else { bad_router[id] = bad_router[id] || is_bad; }
    #print vals[i], " -> "fields[8]" => "is_bad", "bad_router[id];
  }
}
# Verifier records
/V [[:alpha:]] [0-9]+/ {
  match(RT, /[0-9]+/, vals); id = strtonum(vals[0]);
  patsplit($0, vals, /V [[:alpha:]] [0-9]+/);
  if (length(vals) != face_ops[id][3]) { print "["id"]\tVerifier iterations mismatch"; exit; }

  for (i in vals) {
    split(vals[i], fields, " ");
    
    verify_records[id][i][1] = fields[2] # exit status
    verify_records[id][i][2] = fields[3] # time
  }
}
END {
  # Placer instance stats
  placer_time_total = 0;
  placer_time_instance_min = min_init_val;
  placer_time_instance_max = max_init_val;
  placer_num_instances = 0;
  for (id in placer_records) { # fid
    for (i in placer_records[id]) { # instance
      time = placer_records[id][i][2]
      time_face[id] += time;
      placer_time_total += time;

      placer_num_instances++;
      if (time < placer_time_instance_min) {
        placer_time_instance_min = time;
      }
      if (time > placer_time_instance_max) {
        placer_time_instance_max = time;
      }
    }
  }
  placer_time_instance_avg = placer_time_total / placer_num_instances;

  # Router instance stats
  router_time_total = 0;
  router_time_instance_min = min_init_val;
  router_time_instance_max = max_init_val;
  router_num_instances = 0;
  for (id in router_records) {
    for (i in router_records[id]) {
#      time = router_records[id][i][7];
      time = router_records[id][i][9];
      time_face[id] += time;
      router_time_total += time;

      router_num_instances++;
      if (time < router_time_instance_min) {
        router_time_instance_min = time;
      }
      if (time > router_time_instance_max) {
        router_time_instance_max = time;
      }

      if (router_records[id][i][1] == "h") {
        failure_router_iospace[id]++;
      } else if (router_records[id][i][1] == "i") {
        failure_router_inter[id]++;
      } else if (router_records[id][i][1] == "s") {
        failure_router_space[id]++;
      }
    
    }
  }
  router_time_instance_avg = router_time_total / router_num_instances;

  # Verifier instance stats
  verify_time_total = 0;
  verify_time_instance_min = min_init_val;
  verify_time_instance_max = max_init_val;
  verify_num_instances = 0;
  for (id in verify_records) {
    for (i in verify_records[id]) {
      time = verify_records[id][i][2];
      time_face[id] += time;
      verify_time_total += time;

      verify_num_instances++;
      if (time < verify_time_instance_min) {
        verify_time_instance_min = time;
      }
      if (time > verify_time_instance_max) {
        verify_time_instance_max = time;
      }

      if (verify_records[id][i][1] == "p") {
        failure_verify_plane[id]++;
      } else if (verify_records[id][i][1] == "t") {
        failure_verify_trace[id]++;
      } else if (verify_records[id][i][1] == "b") {
        failure_verify_both[id]++;
      }
    }
  }
  verify_time_instance_avg = verify_time_total / verify_num_instances;

  time_face_total = 0;
  time_face_min = min_init_val;
  time_face_max = max_init_val;
  time_face_avg = 0;
  for (id in time_face) {
    time_face_total += time_face[id];

    if (time_face[id] < time_face_min) {
      time_face_min = time_face[id];
    }
    if (time_face[id] > time_face_max) {
      time_face_max = time_face[id];
    }
  }
  time_face_avg = time_face_total / num_faces;

  failure_total_router_iospace = 0;
  failure_total_router_inter = 0;
  failure_total_router_space = 0;
  failure_total_verify_plane = 0;
  failure_total_verify_trace = 0;
  failure_total_verify_both = 0;
  for (id = 0; id <= max_id; ++id) {
    failure_total_router_iospace += failure_router_iospace[id];
    failure_total_router_inter += failure_router_inter[id];
    failure_total_router_space += failure_router_space[id];
    failure_total_verify_plane += failure_verify_plane[id];
    failure_total_verify_trace += failure_verify_trace[id];
    failure_total_verify_both += failure_verify_both[id];
  }
  failure_total_router = failure_total_router_iospace + failure_total_router_inter + failure_total_router_space;
  failure_total_verify = failure_total_verify_plane + failure_total_verify_trace + failure_total_verify_both;
  failure_total = failure_total_router + failure_total_verify;

  # Sanity check
  pni = rni = vni = 0;
  for (id = 0; id <= max_id; ++id) {
    pni += face_ops[id][1];
    rni += face_ops[id][2];
    vni += face_ops[id][3];
  }
  if ((pni != placer_num_instances) || (rni != router_num_instances) || (vni != verify_num_instances)) {
    print "PRV instance number mismatch"; exit;
  }
  
  theoretical_max_rects = 0;
  for (id = 0; id <= max_id; ++id) {
    max = 0;
    for (i = 1; i <= length(placer_records[id]); ++i) {
      if (placer_records[id][i][1] > max) {
        max = placer_records[id][i][1];
      }
    }
    theoretical_max_rects += max;
  }

  rects_per_face_avg = 0;
  rects_per_face_min = min_init_val;
  rects_per_face_max = max_init_val;
  for (id = 0; id <= max_id; ++id) {
    final_rects = placer_records[id][length(placer_records[id])][1];
    rects_per_face_avg += final_rects;
    
    if (final_rects < rects_per_face_min) {
      rects_per_face_min = final_rects;
    }
    
    if (final_rects > rects_per_face_max) {
      rects_per_face_max = final_rects;
    }
  }
  rects_per_face_avg = rects_per_face_avg / num_faces;
  
  triangle_coverage_min = min_init_val;
  triangle_coverage_max = max_init_val;
  triangle_coverage_avg = 0;
  for (id = 0; id <= max_id; ++id) {
    coverage = 0;
    for (i = 1; i <= length(module_area[id]); ++i) {
      coverage += module_area[id][i];
    }
    for (j = 1; j <= length(connector_area[id]); ++j) {
      coverage += connector_area[id][j];
    }
    ratio = coverage / triangle_area[id];
    
    if (ratio < triangle_coverage_min) {
      triangle_coverage_min = ratio;
    }
    if (ratio > triangle_coverage_max) {
      triangle_coverage_max = ratio;
    }
    triangle_coverage_avg += ratio;
  }
  triangle_coverage_avg /= num_faces;
  
  full_hinges_total = 0;
  half_hinges_total = 0;
  for (id = 0; id <= max_id; ++id) {
    full_hinges_total += num_hinges_all[id] - num_hinges_half[id];
    half_hinges_total += num_hinges_half[id];
  }
  # all hinges are counted twice (seen from each triangle)
  full_hinges_total /= 2;
  half_hinges_total /= 2;
  
  for (id = 0; id <= max_id; ++id) {
    rects = placer_records[id][length(placer_records[id])][1];
    subseq_stats[rects][len = length(subseq_stats[rects])][1] = router_records[id][length(placer_records[id])][8]; # max len subseq
    subseq_stats[rects][len][2] = router_records[id][length(placer_records[id])][9]; # max_len_subseq
  }

  printf "\n################################################################################\n"
  printf "File: %s\n", ARGV[1];
  printf "SUMMARY\n";
  printf "Number of faces: %d\n", num_faces;
  printf "Hinge split (full / half): %d / %d\n", full_hinges_total, half_hinges_total;
  printf "Theoretical max rects: %d\n", theoretical_max_rects;
  printf "Module loss percentage: %.1f%%\n", 100 * (theoretical_max_rects - num_connectors - num_modules) / theoretical_max_rects;
  printf "Number of modules / connectors: %d / %d\n", num_modules, num_connectors;
  printf "Number of rects per face (min / max / avg): %d / %d / %.1f\n", rects_per_face_min, rects_per_face_max, rects_per_face_avg;
  printf "Total time: %.3fs\n", time_face_total / 1e3;
  printf "%% of total time (placer / router / verifier): %.1f%% / %.1f%% / %.1f%%\n", 1e2 * placer_time_total / time_face_total, 1e2 * router_time_total / time_face_total, 1e2 * verify_time_total / time_face_total;
  printf "Per face (min / max / avg): %.3fs / %.3fs / %.3fs\n", time_face_min / 1e3, time_face_max / 1e3, time_face_avg / 1e3;
  printf "Times run (placer / router / verifier): %d / %d / %d\n", placer_num_instances, router_num_instances, verify_num_instances;
  #printf "Sanity check (placer / router / verifier): %d / %d / %d\n", pni, rni, vni;
  printf "Failures (router / verifier / total): %d / %d / %d\n", failure_total_router, failure_total_verify, failure_total;
  printf "Coverage ratios (min / max / avg): %.1f%% / %.1f%% / %.1f%%\n", triangle_coverage_min * 1e2, triangle_coverage_max * 1e2, triangle_coverage_avg * 1e2;

  printf "\nPLACER\n";
  printf "Times run: %d\n", placer_num_instances; 
  printf "Total time: %.3fs\n", placer_time_total / 1e3;
  printf "Per instance (min / max / avg): %.3fs / %.3fs / %.3fs\n", placer_time_instance_min / 1e3, placer_time_instance_max / 1e3, placer_time_instance_avg / 1e3;

  printf "\nROUTER\n";
  printf "Times run: %d\n", router_num_instances; 
  printf "Successes / hfails / ifails / sfails: %d / %d / %d / %d\n", router_num_instances - failure_total_router, failure_total_router_iospace, failure_total_router_inter, failure_total_router_space;
  printf "Total time: %.3fs\n", router_time_total / 1e3;
  printf "Per instance (min / max / avg): %.3fs / %.3fs / %.3fs\n", router_time_instance_min / 1e3, router_time_instance_max / 1e3, router_time_instance_avg / 1e3;
  
  printf "\nVERIFIER\n";
  printf "Times run: %d\n", verify_num_instances; 
  printf "Successes / pfails / tfails / bfails: %d / %d / %d / %d\n", verify_num_instances - failure_total_verify, failure_total_verify_plane, failure_total_verify_trace, failure_total_verify_both;
  printf "Total time: %.3fs\n", verify_time_total / 1e3;
  printf "Per instance (min / max / avg): %.3fs / %.3fs / %.3fs\n", verify_time_instance_min / 1e3, verify_time_instance_max / 1e3, verify_time_instance_avg / 1e3;

  printf "\nUNSTABLE SUBSEQS\n";
  arg_max_mlss = 0;
  max_mlss = max_init_val;
  for (fr in subseq_stats) {
    printf "%d\t->\t", fr;
    avg = 0;
    avg_time = 0;
    cnt = 0;
    for (mlss in subseq_stats[fr]) {
      val = subseq_stats[fr][mlss][1];
      time = subseq_stats[fr][mlss][2];
      if (val >= 0) {
        if (val > max_mlss) { max_mlss = val; arg_max_mlss = fr; }
        avg += val;
        avg_time += time;
        cnt++;
        printf "[%d, %.3f] ", val, time / 1e3;
      }
    }
    if (cnt > 0) {
      printf "\n%d\t->\t AVG %.1f in %.3fs (n=%d)", fr, avg / cnt, avg_time / (1e3 * cnt), cnt ;
    }
    printf "\n";
  }
  printf "OVERALL MLSS: %d (arg %d)\n", max_mlss, arg_max_mlss;

  # printf "t > %.1fs:\t", time_threshold_ms_placer / 1e3;
  # for (i in bad_placer) {
  #   if (bad_placer[i]) { printf "%d ", i }
  # }
  # printf "t > %.1fs:\t", time_threshold_ms_router / 1e3;
  # for (i in bad_router) {
  #   if (bad_router[i]) { printf "%d ", i }
  # }
  # printf "\n"
}
#/e [0-9]*/ { print $0 }
