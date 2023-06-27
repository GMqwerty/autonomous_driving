import robot_upstart
j = robot_upstart.Job(name="run2")
j.symlink = True
j.add(package="carpkg", filename="scripts/launch/run2.launch")
j.install()