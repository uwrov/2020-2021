#!/usr/bin/env python3

import time

class timer():

    start_time = 0.0
    elapsed_times = []

    def __init__(self):
        self.start_time = 0.0
        self.elapsed_times = [];
    
    def start(self):
        self.start_time = time.time()

        self.elapsed_times.clear()
        self.elapsed_times.append(self.start_time)
    
    def add_elapsed_time(self):
        self.elapsed_times.append(time.time())

    def results(self):
        end = time.time()
        print('Results: ')
        print('Total Time Taken: ' + str((end - self.start_time)))
        for i in range(1, len(self.elapsed_times)):
            print('Section ' + str(i) + ': ' + str((self.elapsed_times[i] - self.elapsed_times[i - 1])))



if __name__ == '__main__':
    '''
    clock = timer();
    clock.start();
    clock.add_elapsed_time();
    print('Yo')
    clock.add_elapsed_time();
    clock.results();
    '''
