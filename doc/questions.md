
1. "30179 segmentation fault  ./particle_filter", how to debug this issue?
first try:
added logs, seems that worked. have no idea what's to do? 
```
Initialize done.
[sense_observations_x, sense_observations_y]: 2.5318 11.3044 -19.8203 1.8477 13.9833 29.8866 -13.5371 30.1787 -36.3185 22.5064 -46.1799 , 5.5906 -6.7468 -2.0902 -23.2643 -21.6063 12.8641 -36.3244 -31.1239 -25.8484 -41.4829 -14.4651
updateWeights done
Resample done
highest w 0.900309
average w 0.14193
get associations start
get associations end
get getSenseX start
get getSenseX end
get getSenseY start
get getSenseY end
42["best_particle",{"best_particle_associations":"31 2 39 38 37 11 3 23 16 15 12","best_particle_sense_x":"8.7638 17.42 -13.641 8.2018 20.139 36.203 -7.1285 36.626 -29.836 28.898 -39.786","best_particle_sense_y":"7.5647 -4.5993 -0.098341 -20.97 -20.15 14.827 -34.54 -28.898 -23.277
-39.754 -12.615","best_particle_theta":0.00182318741027652,"best_particle_x":6.31935846397864,"best_particle_y":1.98453119851265}]
one iteration done
[1]    30179 segmentation fault  ./particle_filter
```

the "segnetation fault" problem is caused by below code:
```
        // s[0] = particles[i].x;
        // s[1] = particles[i].y;
        // s[2] = particles[i].theta;
        // shoud be change to:
        s.push_back(particles[i].x);
        s.push_back(particles[i].y);
        s.push_back(particles[i].theta);
        // ERROR: i originallly used below code to assign values, 
        // this produces the "segment fault" error. 

```

    how about try the code without the simulator? 
get the first version code from github, integrated the main coce without the simulator.

2. question: the `mywork_with_error` branch can run the code without "segment fault", merged this branch to master, why the master has this error?






