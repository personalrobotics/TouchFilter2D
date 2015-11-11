# TouchFilter2D
Experiments with touch in 2D using openframeworks

Requires [OpenFrameworks](http://openframeworks.cc/). It *must* be checked out in your openframeworks apps directory:

`<OF_ROOT>/apps/myApps/`

it also requires `nlopt` installed.

Contains 3 experiments:

* Simple Filter - a simple 3D environment with a point robot and a surface.
    ![img](https://github.com/personalrobotics/TouchFilter2D/blob/master/simple_filter.png?raw=true)
   WASD + Q/E can be used to move the robot around;
* Analytic Filter (IK) - a 2-link 2-joint robot with a single point
  ![img](https://github.com/personalrobotics/TouchFilter2D/blob/master/analytic_filter.png?raw=true)
   Use the mouse to move the robot around.
* Touch Filter - a 3-link 3-joint robot with a signed distance field based environment.
  ![img](https://github.com/personalrobotics/TouchFilter2D/blob/master/sdf_filter.png?raw=true)
   use the mouse to move the robot around

Most of the experiments can be run in the following modes:

* CPF - Conventional particle filter
* MPF_Analytic - Manifold particle filter with an analytic representation of the manifold
* MPF_ParticleProjection - MPF with projection of the current belief onto the manifold
* MPF_UniformProjection - MPF with projection of a uniform sample set onto the manifold
* MPF_BallProjection - MPF which projects a sample from a bounding ball around the current belief
  
