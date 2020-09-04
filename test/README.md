# Tests

all_tests.cpp will run all the tests

## Using catkin_make

```
catkin_make run_tests
```

should give the detailed output

```
[==========] Running 2 tests from 1 test case.
[----------] Global test environment set-up.
[----------] 2 tests from StrCompare
[ RUN      ] StrCompare.CStrEqual
[       OK ] StrCompare.CStrEqual (0 ms)
[ RUN      ] StrCompare.CStrEqual2
[       OK ] StrCompare.CStrEqual2 (0 ms)
[----------] 2 tests from StrCompare (1 ms total)

[----------] Global test environment tear-down
[==========] 2 tests from 1 test case ran. (1 ms total)
[  PASSED  ] 2 tests.
```


## Using Catkin Tools

It is better to use `catkin_make` during development.  See above.

Catkin Tools is useful during builds.

```
catkin run_tests
```

Provides no valuable feedback so you must then run:

```
catkin_test_results
```

which provides a limited summary and exit code

```
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

So look in the test results folder for more details from the XML report:

```
catkin_ws/build/am_super/test_results/am_super
```

## Running Ros Tests Only

```
rostest am_super life_cycle_rostest.launch --text
```

For focus on the ROS Tests, you can run the `rostest` command directly, similar output to `roslaunch`:

```
... logging to /home/developer/.ros/log/rostest-0bbd7f928be4-3823.log
[ROSUNIT] Outputting test results to /home/developer/.ros/test_results/am_super/rostest-rostest_life_cycle_rostest.xml
[ INFO] [1598449033.571462621] /am_super /am_super:AMSuper: 
[ INFO] [1598449033.572523600] /am_super node_timeout_s = 2
[ INFO] [1598449033.573059413] /am_super configuring nodes from manifest: life_cycle_rostest
[ INFO] [1598449033.573102524] /am_super   life_cycle_rostest
[ INFO] [1598449033.573157065] /am_super state: OFF, manifest: 1, manifest online:0, total online:0
[ INFO] [1598449033.593012556] /am_super LG: NO LOG_XXX SD CARD FOUND
[ INFO] [1598449033.593439290] /am_super Opening bag file /var/log/amros/SU-0004-2020-08-26-13-37-13.bag
[ INFO] [1598449033.596636438] /am_super /am_super: running...
[ERROR] [1598449034.597419009] /am_super state: BOOTING, manifest: 1, manifest online:0, total online:0
[ERROR] [1598449034.597601467] /am_super not online: life_cycle_rostest
[ INFO] [1598449035.013772296] /am_super manifested node 'life_cycle_rostest' came online
[ WARN] [1598449035.013921944] /am_super life_cycle_rostest changed process id to = 0
[ INFO] [1598449035.014116936] /am_super state: BOOTING, manifest: 1, manifest online:1, total online:1
[ INFO] [1598449035.014259065] /am_super BOOTING --> READY
[ INFO] [1598449035.014352222] /am_super sending CONFIGURE to all nodes
[ INFO] [1598449035.014652967] /am_super sending command: CONFIGURE
[ INFO] [1598449035.014836112] /am_super state: READY, manifest: 1, manifest online:1, total online:1
[ INFO] [1598449035.597029428] /am_super READY: sending CONFIGURE again
[ INFO] [1598449035.597176324] /am_super sending command: CONFIGURE
[ INFO] [1598449035.597545319] /am_super state: READY, manifest: 1, manifest online:1, total online:1
[ INFO] [1598449036.013297609] /am_super life_cycle_rostest changed state to = CONFIGURING
[ INFO] [1598449036.013454340] /am_super state: READY, manifest: 1, manifest online:1, total online:1
[ INFO] [1598449036.013610387] /am_super READY: sending CONFIGURE again
[ INFO] [1598449036.013696460] /am_super sending command: CONFIGURE
[ INFO] [1598449036.014050425] /am_super life_cycle_rostest changed state to = INACTIVE
[ INFO] [1598449036.014185408] /am_super state: READY, manifest: 1, manifest online:1, total online:1
[ INFO] [1598449046.597354562] /am_super state: READY, manifest: 1, manifest online:1, total online:1
[Testcase: testlife_cycle_rostest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[am_super.rosunit-life_cycle_rostest/testState_SuperRemainsInREADY][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/developer/.ros/log/rostest-0bbd7f928be4-3823.log
```