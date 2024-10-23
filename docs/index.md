<p style="text-align: center;"> 
<strong> The International Journal of Robotics Research (IJRR), 2024 </strong>
</p>

<p style="text-align: center;"> 
Journal version of 
<a href="https://mingrui-yu.github.io/DLO_planning/" style="color: #0ABAB5; text-decoration: underline;">ICRA 2023</a>
</p>

<p style="text-align: center;">

</p>

<p style="text-align: center;"> Tsinghua University, UC Berkeley </p>

<p style="text-align: center;"> 
<a href="https://arxiv.org/abs/2310.09899" style="color: #0ABAB5; text-decoration: underline;">arXiv</a> | 
<a href="https://github.com/Mingrui-Yu/DLO_planning_2" style="color: #0ABAB5; text-decoration: underline;">Code</a>
</p>

## Video

<video controls style="width: 100%; height: auto;">
    <source src="./final_whole.mp4" type="video/mp4">
    Your browser does not support the video tag.
</video>

## Abstract

Constrained environments, compared with open spaces without other objects, are more common in practical applications of manipulating deformable linear objects (DLOs) by robots, where movements of both DLOs and robot manipulators should be constrained and unintended collision should be avoided. This task is high-dimensional and highly constrained owing to the highly deformable DLOs, dual-arm robots with high degrees of freedom, and 3-D complex environments, which render global planning extremely challenging. Furthermore, accurate DLO models needed by planning are often unavailable owing to their strong nonlinearity and diversity, resulting in unreliable planned paths.

This article focuses on the global moving and shaping of DLOs in constrained environments by dual-arm robots. The main objectives are 1) to efficiently and accurately accomplish this task, and 2) to achieve generalizable and robust manipulation of various DLOs.

To this end, we propose a complementary framework with whole-body planning and control using appropriate DLO model representations. First, a global planner is proposed to efficiently find feasible solutions based on a simplified DLO energy model, which considers the full system states and all constraints to plan more reliable paths.
Then, a closed-loop manipulation scheme is proposed to compensate for the modeling errors and enhance the robustness and accuracy, which incorporates a constrained model predictive controller to track the planned path as guidance while real-time adjusting the robot motion based on an adaptive DLO motion model.
This framework systematically considers multiple constraints for this problem, including stable deformation, overstretch prevention, closed-chain movements, and collision avoidance. The key novelty is that it can efficiently solve the high-dimensional problem subject to all those constraints and generalize to various DLOs without elaborate model identifications.

Experiments demonstrate that our framework can accomplish considerably more complicated tasks than existing works. It achieves a 100% planning success rate among thousands of trials with an average time cost of less than 15 second, and a 100% manipulation success rate among 135 real-world tests on five different DLOs.

## Citation

Please cite our paper if you find it helpful :)

```
@article{yu2024generalizable,
  title={Generalizable whole-body global manipulation of deformable linear objects by dual-arm robot in 3-d constrained environments},
  author={Yu, Mingrui and Lv, Kangchen and Wang, Changhao and Jiang, Yongpeng and Tomizuka, Masayoshi and Li, Xiang},
  journal={The International Journal of Robotics Research},
  year={2024}
}
```

## Contact

If you have any question, feel free to contact the authors: Mingrui Yu, [mingruiyu98@gmail.com](mailto:mingruiyu98@gmail.com) .

Mingrui Yu's Homepage is at [mingrui-yu.github.io](https://mingrui-yu.github.io).
