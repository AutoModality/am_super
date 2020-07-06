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

