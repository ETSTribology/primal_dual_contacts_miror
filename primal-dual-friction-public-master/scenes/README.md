# Input format

The scenes in this repository provide correspond to the examples shown in the main paper (plus some extras), and should provide a good sense of how to set up your own rigid body simulation. Continue reading for an explanation of all parameters below.

Some experimental features not documented in the main paper are marked with “:bangbang:”. They either are not thoroughly tested, or may cause simulations to break uexpectedly. Use these features with caution.

## Simulation setup

Generally, a simulation setup must contain one or more GLTF scenes, as well as a YAML config file. Both `.gltf` and `.glb` formats are supported.

> :information_source: NOTE
>
> In theory, the GLTF file(s) can come from any source, but all input scenes in this repository were made and exported with [Blender](https://www.blender.org); no other 3D software are currently tested.

> :warning: WARNING
>
> Blender exports flat-shaded meshes as *triangle soup* rather than *triangle meshes*. Make sure to enable smooth shading for *all* meshes before exporting to avoid unexpected errors! Also, do *not* enable data compression for GLTF scenes while exporting.

## YAML config file

The YAML file is broken into the following four sections:

* `scene`: The scene setup specifying objects and initial positions/velocities.
* `simulation`: Simulation parameters.
* `baking`: Settings regarding simulation exporting.
* `logging`: Set up which parameters to log during simulation.

### `scene`

This contains all settings regarding the scene setup. An example is given below:

```yaml
scene:
    meshes: ["meshes_a.glb", "meshes_b.glb"]
    spheres: ["spheres.glb"] # arrays with only one string are also allowed.
    cubes: "cubes.glb"
    static: [0, 13, 27]
    initialize: [
        {
            id: 1,
            density: 3,
            velocity: [0, 1, 30.2],
            angular_velocity: [13, 4.5, 2]
        },
        {
            id: 12,
            density: 5,
            velocity: [12.3, 2, 10.2],
            angular_velocity: [1, 3, 0.5]
        }
    ]
# ...
```

Its options consist of three catagories:

* **Collision geometry** input (`cubes`, `spheres`, `ellipsoids`, `meshes`, `convex`) contain the file name(s) of the GLTF scene(s) to import into the simulation. *The file path is relative to the location of the YAML file*. Furthermore, each GLTF file can be imported as the following collision geometry:
    * `cubes`: Analytical cuboids.
    * `spheres`: Analytical spheres.
    * `ellipsoids`: Analytical ellipsoids.
    * `meshes`: A *closed, manifold, and orientable* triangle mesh. Contacts are generated using vertex-SDF (Signed Distance Field) collision detection (see [Guendelman et al. 2003](https://physbam.stanford.edu/~fedkiw/papers/stanford2003-01.pdf) for details). As such, our contact solver inherits its limitations, such as lack of edge-edge contact detection and incorrect normal directions for coarse meshes.
        
        > :information_source: NOTE
        > 
        > As expected from SDF based collisions, meshes that do not meet the criteria above, such as thin sheets, thin shells, triangle soups, Klein bottles, etc. are not supported. Using such meshes results in undefined behaviour, and the authors as well as ISTA shall not be responsible for the disorienting collapse of reality in the impending Kleinbottlepocalypse.

    * `convex`:bangbang: : A convex polyhedron, *assumed to be already convex in the GLTF file*. Contact generation is implemented in [FCL](https://github.com/flexible-collision-library/fcl) using GJK/EPA and *generates at most one contact point*. As a result, stable stacking is difficult to achive with convex polyhedra, and it is currently not recommended. [HPP-CFL, a fork of FCL soon to be renamed Coal](https://github.com/humanoid-path-planner/hpp-fcl) aims to address some of the issues by calculating contact patch and providing a faster implementation of GJK. It may be incorporated into our solver in the future.
    
    Each collsion geometry input can be either a string or an array of strings, in which all files in the array are imported. Mixing primitive types is also allowed, as demonstrated above.
    
    The parameters of the analytical shapes are automatically calculated such that both the center of mass and inertia tensor aligns with the input mesh as much as possible. As such, there are no tunable parameters for these options.

* `static` (*optional*) specifies the IDs of rigid bodies which have prescribed motion, i.e. reacts to neither contact nor external forces. If you are unsure of the ID of the rigid body, load the scene in the GUI program `ContactSimulation` and right click on the desired rigid body to display the object ID.
* `initialize` (*optional*) specifies the initial velocity and density of each object. It consists of an array of key-value pairs in the following format:
    * `id`: the object ID to specify.
    * `velocity`: the object's initial linear velocity, as a 3D vector.
    * `density`: the objects's density.
    * `angular_velocity`: the object's initial angular velocity in axis-angle representation as a 3D vector.
    
    For all objects *not* specified in `initialize`, zero velocity and a density of `1` is given. These can also be adjusted in the GUI program.

### `simulation`

This sets up the simulation parameters. Example, with an example below:

``` yaml
# ...
simulation:
  timestep: 0.005
  adaptive_timestep: false
  solver:
    name: "primal_dual"
    iterations: 50
    tolerance: 1e-4
    non_smooth_force:
      name: "non_smooth_contact"
      restitution: 0.0
      static_coeff: 0.6
  gravity: [0, -9.81, 0]
# ...
```

It consists of the following options:
* `timestep`: (*default: `0.01`*) the time step size of the simulation in seconds.
* `adaptive_timestep`: (*default: `false`*) whether to use adaptive timestepping, based on the oriented bounding box size of the rigid bodies as described by [Guendelman et al. 2003](https://physbam.stanford.edu/~fedkiw/papers/stanford2003-01.pdf).
* `substep_factor`: Chooses a substep size that guarantees that no two objects moves closer to each other than `minimum_obb_dimension / (2*substep_factor)`. This option is mandatory if `adaptive_timestep` is set to `true`, and is ignored if set to `false`.
* `solver`: Solver settings. Contains the following options:
    * `name`: The name of the solver to use. Currently only `primal_dual` is implemented, and must be set as such (other names will lead to an error).
    * `iterations`: (*default: `50`*) the maximum number of iterations per timestep.
    * `tolerence`: (*default: `1e-4`*) the error tolerance of the solver before it stops.
    * `non_smooth_force`: The settings of the non-smooth force. See below for options.
    * `gravity`: (*default: `[0, -9.81, 0]`*) The gravity in the scene.

#### `non_smooth_force` settings

* `name`: The name of the non-smooth contact force to use. Possible settings are `only_normal` and `non-smooth_contact`. The options for both largely overlap, the main difference being that `only_normal` lacks friction.
* `spring_based_force`: (*default: `false`*) Toggles Baumgarte stabilization, an extra force to reduce object penetration.
* `spring_k`, `spring_damp`: Typically, Baumgarte stabilization takes in a compliance and (penetration depth) restoration parameter. This however does not behave consistently when adaptive timestepping is enabled. We choose instead to specify Baumgarte stabilization parameters by linear spring stiffness and dampening parameters. For the relation between Baumgarte stabilization and linear springs, refer to the [SIGGRAPH 2022 contact simulation tutorial](https://siggraphcontact.github.io/), [Section 1.9](https://siggraphcontact.github.io/assets/files/SIGGRAPH22_friction_contact_notes.pdf#subsection.1.9).

    > :information_source: NOTE
    > 
    > As larger spring stiffnesses typically require larger dampening for stable simulations, `spring_damp` is represented as a fraction of `spring_k`. A `spring_k` of `1e4` and `spring_damp` of `0.1`, for example, would result in a dampening coefficient of `1e3`, rather than `0.1`.
* `restitution` :bangbang: : (*float from `0.0` to `1.0`*) Replaces the Signorini-Coulomb condition (equivalent to a value of `0.0`) with a Newton collision condition with restitution. This is empirically stable for some simulations, but may suffer from interpenetration, long force chains characteristic of projected Gauss-Seidel methods in some methods such as in the scene `cube_stack` from the examples. For a detailed explanation of the difficulties of simulation restitution, read [*Reflections on Simultaneous Impact*](https://www.cs.columbia.edu/cg/rosi/) by Smith et al. 2012. This option is ignored when `spring_based_force` is set to `true`.
* `static_coeff`: (*exclusive to `non_smooth_contact`*) The friction coefficient.

### `baking`

This section controls options for exporting the simulated results. It consists of the following options:

* `time`: (*default: `10.0`*) The total simulation time in seconds.
* `output_collision_geometry` :bangbang: : (*default: `true`*) When set to true, the collision geometries are exported in the final simulation. For example, a GLTF scene set to `ellipsoids` under `scene` will have each object converted to ellipsoids in the final exported scenes. If you wish to keep the geometry from the input GLTF instead set this to `false`.

### `logging`

This controls the amount of information output to terminal during simulation. Currently possible settings are either `"all"` (log everything) or `"none"` (don't log anything). More fine-grain adjustments can be found under "Logging" in the GUI program.

