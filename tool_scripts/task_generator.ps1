Write-Host “Script written by huangjia2hi@163.com"
Write-Host "请输入任务名"
$Task_Name = Read-Host

$C_Code = @"
 /* Definitions for $Task_Name */
 osThreadId_t ${Task_Name}Handle;
 uint32_t ${Task_Name}Buffer[128];
 osStaticThreadDef_t ${Task_Name}ControlBlock;
 const osThreadAttr_t ${Task_Name}_attributes = {
     .name = "$Task_Name",
     .cb_mem = &${Task_Name}ControlBlock,
     .cb_size = sizeof(${Task_Name}ControlBlock),
     .stack_mem = &${Task_Name}Buffer[0],
     .stack_size = sizeof(${Task_Name}Buffer),
     .priority = (osPriority_t) osPriorityNormal,
 };
"@

Write-Host "任务模版已生成"
Write-Host $C_Code

Set-Clipboard $C_Code