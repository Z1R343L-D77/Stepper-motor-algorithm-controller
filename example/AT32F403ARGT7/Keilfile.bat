@echo off
echo 正在删除KEIL编译的中间文件

del *Backup?of*.* /s
del *.ldb /s

del *.bak /s /f
del *.ddk /s /f
del *.edk /s /f
del *.lst /s /f
del *.lnp /s /f
del *.mpf /s /f
del *.mpj /s /f
del *.obj /s /f
del *.omf /s /f
del *.opt /s /f
del *.plg /s /f
del *.rpt /s /f
del *.tmp /s /f
del *.__i /s /f
del *.crf /s /f
del *.o /s /f
del *.d /s /f
del *.axf /s /f
del *.tra /s /f
del *.dep /s /f
del *._ia /s /f
del *.pbi /s /f
del *.cout /s /f
del *.pbd /s /f
del *.browse /s /f
del JLink*.* /s /f
del EventRecorderStub*.* /s /f
del ArInp.Scr /s /f

rd /s /q DebugConfig
rd /s /q APP_OUT
rd /s /q History

del *.iex /s /f
del *.htm /s /f
del *.sct /s /f
del *.map /s /f
del *.bkp /s /f

del *.pdb /s /f
del *.ilk /s /f
del *.pch /s /f
del *.ncb /s /f

del *.~c /s /f
del *.~h /s /f

del *.PCBDOCPreview /s /f
del *.SchDocPreview /s /f
del *.PRJPCBStructure /s /f
del *.$$$Preview /s /f
del *.Annotation /s /f
del *.AnnotationPreview /s /f

del *.VC.db /s /f


echo 清除系统LJ完成！
exit
