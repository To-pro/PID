# PID.h
此函示庫是用來實現PID控制器

## 使用說明:

控制器輸出 $u$ 為

$u = \alpha ( \beta - k_p  e - k_i  \int_0^t e(\tau)d\tau - k_d  \dot{e})$

其中 $\alpha,\beta$ 為每次更新控制律要給定的值(默認$\alpha = 1,\beta = 0$)。
並且 $e$ 為追蹤誤差， $\dot{e}$ 可給可不給 ( 沒有給則插分 $e$ 來估計 $\dot{e}$)。

飽和保護為:

$|u| < 0.99 u_{max}$

99% 是因為 double 會有精度問題(12.xxxxx < 12.xxxxxx)，導致沒有保護到飽和。

### 初始化:
宣告PID控制器物件
``` c
PID controller
``` 
並且使用前要初始化控制器增益(Arduino 中的 Setup 中)，可以給極點位置計算控制增益
``` c
controller.Init_pole(double lambda1,double lambda2 ,double lambda3 ,double u_max_set);
//or
controller.Init_pole(double lambda1,double lambda2 ,double u_max_set);
```
或直接給控制增益
``` c
controller.Init_gain(double kp_set,double ki_set ,double kd_set ,double u_max_set);
//or
controller.Init_gain(double kp_set ,double kd_set ,double u_max_set);
```
若是PD控制就不用設計 $k_i$。另外積分飽和保護預設是遇限削弱法，更改方式為
``` c
controller.intergal_anti_windup_flag_set(int flag_set)
```
其中其他方法為
``` c
enum{
    no_anti_windup,
    separation,//積分分離(intergal_windup_epsilon 需要設定)
    clamping,//積分遇限削弱法
    back_calculation//反饋抑制抗飽和(Kb需要設定)
};
```
### 更新與輸出:
接著就是在控制器中更新控制律(控制器內部中斷中)
``` c
controller.Updata(double e_set,double alpha,double beta);
//or
controller.Updata(double e_set)
//or
controller.Updata(double e_set ,double de_set ,double alpha,double beta)
//or
controller.Updata(double e_set,double de_set)
```
若是每有給 de_set 就是向後差分估計誤差 e_set 對時間的微分。alpha 默認 1，beta 默認 0。

若要將控制律結果輸出則是
``` c
u = controller.Output();
//or
controller.Output(double *pte_u);
``` 
### 補充:
要歸零控制器變數(不包含控制增益，要更改控制增益直接使用 Init_pole 或是 Init_gain)
``` c
controller.Reset();
``` 
默認誤差為實際狀態減去期望狀態(error_define = 1($x-x_d$))，若是要相反控制增益將差一個負號，
``` c
error_define_set(int error_define_set)
``` 
其中error_define為 1 : $e =x-x_d$ ，若是為 0 : $e =x_d-x $

## 範例:
考慮一個系統為

$m \ddot{x} = u$

其中$m$為系統質量、 $x$ 為位置和 $u$ 為輸入。給定追蹤目標 $x_d$，定義追蹤誤差為

$\begin{matrix} e_1 = x - x_d \\ e_2 = \dot{x} - \dot{x}_d\end{matrix}$

設計控制律為

$u = m ( \ddot{x}_d  - k_p  e_1 - k_i  \int_0^t e(\tau)d\tau - k_d  e_2)$

那麼將控制律帶到系統中可以得到

$m \ddot{x} = m ( \ddot{x}_d  - k_p  e_1 - k_i  \int_0^t e(\tau)d\tau - k_d  e_2)$

整理得到

$\ddot{x} - \ddot{x}_d=   - k_p  e_1 - k_i  \int_0^t e(\tau)d\tau - k_d  e_2$

定義 $ \dot{e}_2 = e_3$ 則

誤差動態方程可以表示為

$
\begin{bmatrix}
\dot{e}_1 \\ \dot{e}_2\\ \dot{e}_3
\end{bmatrix}=
\underbrace{\begin{bmatrix}
0 &1 &0 \\ 0 &0 &1 \\ - k_i & - k_p & - k_d
\end{bmatrix}}_{\mathbf{A}_e}
\begin{bmatrix}
e_1 \\ e_2\\ e_3
\end{bmatrix}
$

則系統誤差特徵方程為

$det(\lambda \mathbf{I}-\mathbf{A}_e) = \lambda^3 + k_d \lambda^2 +k_p \lambda + k_i$

若將極點配置在 $\lambda_1 = 1$ 、 $\lambda_2 = 2$ 和 $\lambda_3 = 3$ 則期望系統誤差特徵方程為

$(\lambda + \lambda_1)(\lambda + \lambda_2)(\lambda + \lambda_3)=
\lambda^3 + (\lambda_1 +\lambda_2 +\lambda_3) \lambda^2 +(\lambda_1\lambda_2+\lambda_1\lambda_3 + \lambda_2 \lambda_3) \lambda + \lambda_1 \lambda_2 \lambda_3$

比較係數得到

$k_p = \lambda_1\lambda_2+\lambda_1\lambda_3 + \lambda_2 \lambda_3$

$k_i = \lambda_1 \lambda_2 \lambda_3$

$k_d = \lambda_1 +\lambda_2 +\lambda_3$

此範例要在程式中實現控制律則是:

初始化的程式碼:
``` c
PID controller;
controller..intergal_anti_windup_flag_set(clamping);//此範例設定即默認可省略
controller.error_define_set(1);//此範例設定即默認可省略
controller.Init_pole(1.0,2.0,3.0,12.0);
```
其中假設$|u|\leq 12.0$。因為部分設定是默認可以畫簡成
``` c
PID controller;
controller.Init_pole(1.0,2.0,3.0,12.0);
```

控制器內部中斷的程式碼:
``` c
e_set = x - x_d;
de_set = dx - dx_d;
controller.Updata(e_set,de_set ,m, ddx_d);
u = controller.Output();
```
假設可以拿到速度 (dx) 與位置 (x)，並且期望位置(x_d)、期望速度(dx_d)和期望加速度(ddxd)也可以另外計算得到。

若是要歸零變數(歸零誤差積分)
``` c
controller.Reset();
``` 