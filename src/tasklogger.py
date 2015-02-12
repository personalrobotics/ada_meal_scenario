import json
import time


class TaskLogger(object):

    """Formats task log messages for transmission in ros"""

    def __init__(self, parent=None):
        super(TaskLogger, self).__init__()
        self._subtasks = []
        self._json = {"text": "ROOT",
                      "id": "t0",
                      "completed": 0,
                      "percent": -1.0,
                      "time_left": -1.0,
                      "start_time": -1.0,
                      "end_time": -1.0,
                      "icon": ""
                      }
        self._json_updated = False
        self._parent = parent
        self._callback = None
        self._lastid = 0

    def _nextid(self):
        if self._parent:
            return self._parent._nextid()
        else:
            self._lastid += 1
            return self._lastid

    def _get_json_object(self):
        return self._json

    def set_update_callback(self, callback):
        self._callback = callback

    def get_json(self):
        return json.dumps(self._get_json_object())

    def add_task(self, text="", icon=""):
        newtask = TaskLogger(parent=self)
        newtask._json["text"] = text
        newtask._json["id"] = "t%d" % self._nextid()
        newtask._json["icon"] = icon
        newtask._json["start_time"] = time.time()
        self._subtasks.append(newtask)
        self._update_json()
        self._notify_updated()
        return newtask

    def remove_task(self, task):
        if task in self._subtasks:
            self._subtasks.remove(task)
            self._update_json()
            self._notify_updated()

    def remove_all(self):
        for task in self._subtasks:
            task._parent = None
        self._subtasks = []
        self._update_json()
        self._notify_updated()

    def clear(self):
        self.remove_all()

    def remove(self):
        if self._parent:
            self._parent.remove_task(self)

    def text(self, text):
        self._json["text"] = text
        self._notify_updated()
        return self

    def finish(self, completed):
        self._json["completed"] = completed
        self._json["end_time"] = time.time()
        self._notify_updated()
        return self

    def fail(self):
        return self.finish(-1)

    def succeed(self):
        return self.finish(1)

    def is_complete(self):
        return self._json["completed"] != 0

    def clear_completed(self):
        self._subtasks = [task.clear_completed() for task in self._subtasks
                          if not task.is_complete()]
        self._update_json()
        return self

    def progress(self, percent=-1.0, time_left=-1.0):
        self._json["percent"] = percent
        self._json["time_left"] = time_left
        self._json["completed"] = 0
        self._notify_updated()
        return self

    def _notify_updated(self):
        if self._parent:
            self._parent._notify_updated()
        elif self._callback:
            self._callback(self.get_json())

    def _update_json(self):
        self._json["subtasks"] = []
        for subtask in self._subtasks:
            self._json["subtasks"].append(subtask._get_json_object())

    def __str__(self):
        return self.get_json()

if __name__ == '__main__':
    # Example usages:
    tasklist = TaskLogger()
    task0 = tasklist.add_task(text="Example Task 0")
    task1 = tasklist.add_task(text="Another task")
    task0.succeed()
    task1.progress(percent=50.0)
    task1.add_task(text="A subtask").succeed()
    task1.add_task(text="Another subtask")
    task1.add_task(text="a failed subtask").fail()
    print(str(tasklist))  # equivalent to print(tasklist.get_json())
    tasklist.clear_completed()
    print(str(tasklist))
    tasklist.clear()
