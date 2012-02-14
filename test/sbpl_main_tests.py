#!/usr/bin/env python

from sys import argv
from subprocess import call
from os import getcwd, chdir, pardir, devnull
from os.path import join, exists, abspath

# This script should only be run from sbpl/test/
sbpl_root = abspath(pardir)

def generate_makefile(dir=''):
    """
    Generates a Makefile for SBPL if one doesn't exist

    Looks in a directory relative to the current directory for a Makefile. A new one is generated
    using CMake if one isn't already there. CMakeLists.txt must exist in that directory for this to
    work.

    @return Whether or not a Makefile was generated
    """
    # try to generate Makefile if one doesn't exist
    cwd = join(getcwd(), dir)
    print 'Looking for Makefile in', cwd
    if not exists(join(cwd, 'Makefile')):
        if not exists(join(cwd, 'CMakeLists.txt')):
            return False
        else:
            print 'No Makefile found for SBPL, running cmake'
            call(['cmake', '.'])
        return exists(join(cwd, 'Makefile'))
#end generate_makefile

def run_sbpl_test(env_type, planner_type, test_env, mprim, is_forward_search, navigating=False):
    """
    @brief run the sbpl test executable
    """
    sbpl_exe = join(sbpl_root, 'bin/test_sbpl')

    devnull_fd = open(devnull) # for surpressing output

    test_env_path = join(sbpl_root, test_env)
    mprim_path = join(sbpl_root, mprim)
    print
    print 'Running', planner_type, 'planner on', env_type, 'environment'
    print 'Navigating =', navigating
    print 'Test environment =', test_env
    print 'Motion primitives =', mprim

    forward_search_arg = ''
    if is_forward_search:
        forward_search_arg = 'forward'
    else:
        forward_search_arg = 'backward'

    args = [sbpl_exe, '--env=' + env_type, '--planner=' + planner_type, '--search-dir=' + forward_search_arg, test_env_path, mprim_path]
    if mprim == '': args.pop()
    if navigating: args.insert(1, '-s')
    for arg in args: print arg,
    print

    import time	
    start_time = time.time()

    sbpl_res = call(args, stdout=devnull_fd, stderr=devnull_fd)

    end_time = time.time()
    print 'Planning took', end_time - start_time, 'seconds.'

    green_color = '\033[92;1m'
    red_color = '\033[91;1m'
    end_color = '\033[0m'
    if sbpl_res == 0:
        print green_color + 'Planning succeeded.' + end_color
    else:
        print red_color + 'Planner failed with exit code' + end_color, sbpl_res
    print

    devnull_fd.close()

    return sbpl_res
#end run_sbpl_test

if __name__ == '__main__':
    print "SBPL is located at", sbpl_root

    chdir(sbpl_root)

    makefile_exists = generate_makefile()

    make_result = 0
    # build SBPL
    if makefile_exists:
        if 'rebuild' in argv: call(['make', 'clean'])
        make_result = call(['make'])
    else:
        print 'No Makefile or CMakeLists.txt found. Attempting to run tests without building'

    if make_result != 0:
        print 'Errors building SBPL. Checking for older version of SBPL...'


    sbpl_exists = exists(join(sbpl_root, 'bin/test_sbpl')) and \
                  exists(join(sbpl_root, 'lib/libsbpl.so'))

    if not sbpl_exists:
        print 'Could not build SBPL and SBPL is not already pre-built. Aborting tests'
        exit()
    else:
        print 'SBPL library and test executable built. Proceeding with tests.'

    num_2d_test_successes = 0

    ### PLANNING FOR 2D ENVIRONMENTS ###

    # all planners on 2d environment (12 tests) env1
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1

    # all planners navigating on 2d env1
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1

    # all planners on 2d env2
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_2d_test_successes = num_2d_test_successes + 1

    # all planners navigating on 2d env2 (no thanks, I want my tests to finish)
    #run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True)
    #run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True)
    #run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True)
    #run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', True)

    ### PLANNING FOR (X,Y,THETA) ENVIRONMENTS (9 tests) ###

    num_xytheta_test_successes = 0

    # all planners on xytheta env1
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1

    # all planners navigating on xytheta env1
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True, True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1

    # all planners on xytheta env2
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True) == 0:
        num_xytheta_test_successes = num_xytheta_test_successes + 1

    # all planners navigating on xytheta env2 (no thanks, i want my tests to finish)
    #run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True)
    #run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True)
    #run_sbpl_test('xytheta', 'anstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True)

    ### PLANNING FOR (X,Y,THETA,LEV) ENVIRONMENTS (6 tests) ###

    num_xythetamlev_test_successes = 0

    # all planners on xythetamlev env1
    if run_sbpl_test('xythetamlev', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', True) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1

    # all planners on xythetamlev env2
    if run_sbpl_test('xythetamlev', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'anastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', True) == 0:
        num_xythetamlev_test_successes = num_xythetamlev_test_successes + 1

    ### PLANNING FOR ROBARM ENVIRONMENTS (12 tests) ###

    num_robarm_test_successes = 0

    # all planners on robarm env1
    if run_sbpl_test('robarm', 'arastar', 'env_examples/robarm/env1_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'adstar', 'env_examples/robarm/env1_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'anastar', 'env_examples/robarm/env1_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'rstar', 'env_examples/robarm/env1_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    
    # all planners on robarm env2
    if run_sbpl_test('robarm', 'arastar', 'env_examples/robarm/env2_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'adstar', 'env_examples/robarm/env2_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'anastar', 'env_examples/robarm/env2_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'rstar', 'env_examples/robarm/env2_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1

    # all planners on robarm env3
    if run_sbpl_test('robarm', 'arastar', 'env_examples/robarm/env3_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'adstar', 'env_examples/robarm/env3_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'anastar', 'env_examples/robarm/env3_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1
    if run_sbpl_test('robarm', 'rstar', 'env_examples/robarm/env3_6d.cfg', '', True) == 0:
        num_robarm_test_successes = num_robarm_test_successes + 1


    ###### RUN ALL TESTS WITH BACKWARD SEARCH NOW ######

    ### PLANNING FOR 2D ENVIRONMENTS ###

    num_b_2d_test_successes = 0

    # all planners on 2d environment (12 tests) env1
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1

    # all planners navigating on 2d env1
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, True) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, True) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, True) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env1.cfg', 'matlab/mprim/pr2.mprim', False, True) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1

    # all planners on 2d env2
    if run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1
    if run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_2d_test_successes = num_b_2d_test_successes + 1

    # all planners navigating on 2d env2 (no thanks, I want my tests to finish)
    #run_sbpl_test('2d', 'arastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False)
    #run_sbpl_test('2d', 'adstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False)
    #run_sbpl_test('2d', 'anastar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False)
    #run_sbpl_test('2d', 'rstar', 'env_examples/nav2d/env2.cfg', 'matlab/mprim/pr2.mprim', False)

    ### PLANNING FOR (X,Y,THETA) ENVIRONMENTS (9 tests) ###

    num_b_xytheta_test_successes = 0

    # all planners on xytheta env1
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1

    # all planners navigating on xytheta env1
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False, False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1

    # all planners on xytheta env2
    if run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1
    if run_sbpl_test('xytheta', 'anastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False) == 0:
        num_b_xytheta_test_successes = num_b_xytheta_test_successes + 1

    # all planners navigating on xytheta env2 (no thanks, i want my tests to finish)
    #run_sbpl_test('xytheta', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False)
    #run_sbpl_test('xytheta', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False)
    #run_sbpl_test('xytheta', 'anstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False)

    ### PLANNING FOR (X,Y,THETA,LEV) ENVIRONMENTS (6 tests) ###

    num_b_xythetamlev_test_successes = 0

    # all planners on xythetamlev env1
    if run_sbpl_test('xythetamlev', 'arastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'adstar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'anastar', 'env_examples/nav3d/env1.cfg', 'matlab/mprim/pr2.mprim', False) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1

    # all planners on xythetamlev env2
    if run_sbpl_test('xythetamlev', 'arastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'adstar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1
    if run_sbpl_test('xythetamlev', 'anastar', 'env_examples/nav3d/env2.cfg', 'matlab/mprim/pr2_10cm.mprim', False) == 0:
        num_b_xythetamlev_test_successes = num_b_xythetamlev_test_successes + 1

    print '\033[96;1m', 'Forward search results', '\033[0m'
    print '\033[96;1m', '----------------------', '\033[0m'
    print '\033[96;1m', num_2d_test_successes, 'out of', 12, '2d tests succeeded', '\033[0m'
    print '\033[96;1m', num_xytheta_test_successes, 'out of', 9, 'xytheta tests succeeded.', '\033[0m'
    print '\033[96;1m', num_xythetamlev_test_successes, 'out of', 6, 'xythetamlev tests succeeded.', '\033[0m'
    print '\033[96;1m', num_robarm_test_successes, 'out of', 12, 'robarm tests succeeded.', '\033[0m'

    num_tests = 39
    print '\033[96;1m', num_2d_test_successes + num_xytheta_test_successes + num_xythetamlev_test_successes + \
          num_robarm_test_successes, 'out of', num_tests, 'tests succeeded.', '\033[0m'

    print
    print '\033[96;1m', 'Backward search results', '\033[0m'
    print '\033[96;1m', '-----------------------', '\033[0m'
    print '\033[96;1m', num_b_2d_test_successes, 'out of', 12, '2d tests succeeded', '\033[0m'
    print '\033[96;1m', num_b_xytheta_test_successes, 'out of', 9, 'xytheta tests succeeded.', '\033[0m'
    print '\033[96;1m', num_b_xythetamlev_test_successes, 'out of', 6, 'xythetamlev tests succeeded.', '\033[0m'

    num_b_tests = 27
    print '\033[96;1m', num_b_2d_test_successes + num_b_xytheta_test_successes + num_b_xythetamlev_test_successes, \
          'out of', num_b_tests, 'tests succeeded.', '\033[0m'
#end main

# NOTES
# xytheta and xythetamlev environments do not support R* planning
# envrobarm does not support backward search
