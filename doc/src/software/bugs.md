# Issues


The rover should not be left unchecked for long periods of time without further testing. The testing required to ensure reliable autonomous operation for months at a time is beyond the scope of this project. We do not have time to test what every possibility would happen and what could go wrong over that period. 

The following is a list of issues that we will likely be unable to address due to the current design:
- The rover will not be able to account for all kinds of terrain. Detecting water would require the addition of a completely new processing pipeline that most likely would require deep neural networks. 
- The rover will not account for terrain issues like mud or unstable terrain which may be completely invisible using conventional sensing methods. 
- The rover is not designed with redundancy for sensor failures. The budget and time required for high level redundancy is outside the scope of this project. 
- The rover will be susceptible to external forces (animals, falling things, people, machines). If something were to, for instance, push the robot over there is no recovery plan that the rover can execute.

Due to the above reasons, we also recommend to keep in mind that the obstacle avoidance is a backup at best. We recommend that you **do not** rely on it working. 