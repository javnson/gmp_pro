这个工程是针对电设专用的STM32F407核心板准备的。

由于芯片和硬件的限制，STM32的TIM1并不能触发ADC的SOC，所以需要绕弯路，具体的路径如下：

使用TIM3与TIM1同步，之后使用TIM3触发ADC的SOC。

接下来需要正确启动STM32的PWM和ADC设置

``` C
  /* USER CODE BEGIN 2 */

	// Start Timer 3，从机必须先于主机启动。
	HAL_TIM_Base_Start_IT(&htim3);
	
	//Start Timer 1，主机必须要带_IT结尾
	HAL_TIM_Base_Start_IT(&htim1);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	
	// Start ADC Conversion，执行这一句才能触发ADC中断
	HAL_ADC_Start_DMA(&hadc1, adc_results,6);
	
  /* USER CODE END 2 */

```



为了让GMP能够运行在STM32的环境中，需要做如下的配置：

增加include path

增加GMP相关的源文件

如果需要GMP直接托管项目，可以使用函数`gmp_base_entry()`，
此外需要在主中断中调用`gmp_base_ctl_step()`。


如果不需要GMP直接托管项目，对于STM32的应用场景，
在进入主循环之前调用`gmp_base_init()`
可以在主循环中额外调用`gmp_base_loop()`,
在主中断（ADC转换完成回调）中调用`gmp_base_ctl_step()`。


