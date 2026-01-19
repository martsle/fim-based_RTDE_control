from multiprocessing import Process, Queue, current_process, Value
import time
import queue # imported for using queue.Empty exception

class A:
    def __init__(self):
        self.value = 0

    def do_job(self, tasks_to_accomplish, tasks_that_are_done):
        while True:
            try:
                '''
                    try to get task from the queue. get_nowait() function will 
                    raise queue.Empty exception if the queue is empty. 
                    queue(False) function would do the same task also.
                '''
                self.value += 1
                task = tasks_to_accomplish.get_nowait()
            except queue.Empty:

                break
            else:
                '''
                    if no exception has been raised, add the task completion 
                    message to task_that_are_done queue
                '''
                print(task)
                tasks_that_are_done.put(task + ' is done by ' + current_process().name + ' with value: ' + str(self.value))
                time.sleep(.5)
        return True


def main():
    number_of_task = 100
    number_of_processes = 16
    tasks_to_accomplish = Queue()
    tasks_that_are_done = Queue()
    val = Value('b', True)
    processes = []
    test = A()

    for i in range(number_of_task):
        tasks_to_accomplish.put("Task no " + str(i))

    # creating processes
    for w in range(number_of_processes):
        p = Process(target=test.do_job, args=(tasks_to_accomplish, tasks_that_are_done))
        processes.append(p)
        p.start()

    # completing process
    for p in processes:
        p.join()

    # print the output
    while not tasks_that_are_done.empty():
        print(tasks_that_are_done.get())

    return True


if __name__ == '__main__':
    main()