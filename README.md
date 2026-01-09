# 2026-Robot

## Driver Controls:

```     
           Fast                           
           Mode                          XXXXXX
             \                             /  
      XXXXXX  \                           /   XXXXX
          \__  \__         Gyro Reset  __/  ___/
             \ |  |               \   |  | /
   Drive     _##LB##_   Interrupt  \_##RB##_
      \     /  ___   \_______\_____/\   Y---\---- XXXXX
       \___/__/ . \           \      `X   B--\------XXXXX    
          /   \___/      o (X) o        A-----\-- XXXXX   
         /    ^                          ___   \  
        |   <   >     _____________     / . \___|___,- Turn
 XXXXX -|-----v      /             \    \___/   |
         \__________/               \__________/
```

| Button | Function |
|-|-|
| Start | Interrupt |
| LT | *FAST MODE* |
| LS | Drive |
| RS | Turn |
| X  | Gyro Reset |