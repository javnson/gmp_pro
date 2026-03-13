# processing management

This foldder provide a set of processing management tool kits, including timing manager (not for realtime control),
workflow (high level transfer flow manager), state machine and scheduling.


## Workflow Module of GMP

This is main feature of GMP library. 
This module provide user a easiest way to communicate and implement a long-term delay.
User may use this module stand alone, or user may use this module together with a workflow scheduling.
Generally, if you have a set of workflow to parallel process, we strongly suggest you to use the scheduling.

The workflow itself has the following three key points that should be emphasized.

+ Generally Path of Workflow Class

In this section, here's a one-section workflow, which is shown in the following figure.

<img src="../manual/img/fig1-basic workflow model.jpg" style="zoom: 50%;" />

把start和end做成实体放在全局


+ The General Structure of Section 




+ Special Section Nodes


## Non blocking scheduler

This scheduler is for driving State Machine or Workflow.






