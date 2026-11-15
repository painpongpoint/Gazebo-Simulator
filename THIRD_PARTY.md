# Third-party model assets

The hatchback meshes and textures originate from OpenRobotics models on Gazebo Fuel:

- [Hatchback blue, version 5](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback%20blue/5)
- [Hatchback, version 3](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback/3)

The upstream Hatchback blue model credits Nate Koenig as its author.

`build_car.py` splits and recenters the source mesh into a body and four wheels.
The collision geometry, joints, and control scripts are defined in this project.
Third-party assets retain their upstream licensing; this project does not
relicense them. Consult the source model metadata for their applicable terms.
