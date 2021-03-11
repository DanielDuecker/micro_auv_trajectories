##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=C__
ConfigurationName      :=Debug
WorkspacePath          :=/home/chh/RapidQuadrocopterTrajectories/C++
ProjectPath            :=/home/chh/RapidQuadrocopterTrajectories/C++
IntermediateDirectory  :=$(ConfigurationName)
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=chh
Date                   :=15/06/20
CodeLitePath           :=/home/chh/.codelite
LinkerName             :=/usr/bin/x86_64-linux-gnu-g++
SharedObjectLinkerName :=/usr/bin/x86_64-linux-gnu-g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="C__.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  -O0
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). 
IncludePCH             := 
RcIncludePath          := 
Libs                   := 
ArLibs                 :=  
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch). $(LibraryPathSwitch)Debug 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/x86_64-linux-gnu-ar rcu
CXX      := /usr/bin/x86_64-linux-gnu-g++
CC       := /usr/bin/x86_64-linux-gnu-gcc
CXXFLAGS := -Wall -g -Wall $(Preprocessors)
CFLAGS   :=   $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/x86_64-linux-gnu-as


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/SingleAxisTrajectory.cpp$(ObjectSuffix) $(IntermediateDirectory)/RapidTrajectoryGenerator.cpp$(ObjectSuffix) $(IntermediateDirectory)/Demo.cpp$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@test -d $(ConfigurationName) || $(MakeDirCommand) $(ConfigurationName)


$(IntermediateDirectory)/.d:
	@test -d $(ConfigurationName) || $(MakeDirCommand) $(ConfigurationName)

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/SingleAxisTrajectory.cpp$(ObjectSuffix): SingleAxisTrajectory.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/SingleAxisTrajectory.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/SingleAxisTrajectory.cpp$(DependSuffix) -MM SingleAxisTrajectory.cpp
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/chh/RapidQuadrocopterTrajectories/C++/SingleAxisTrajectory.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/SingleAxisTrajectory.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/SingleAxisTrajectory.cpp$(PreprocessSuffix): SingleAxisTrajectory.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/SingleAxisTrajectory.cpp$(PreprocessSuffix) SingleAxisTrajectory.cpp

$(IntermediateDirectory)/RapidTrajectoryGenerator.cpp$(ObjectSuffix): RapidTrajectoryGenerator.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/RapidTrajectoryGenerator.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/RapidTrajectoryGenerator.cpp$(DependSuffix) -MM RapidTrajectoryGenerator.cpp
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/chh/RapidQuadrocopterTrajectories/C++/RapidTrajectoryGenerator.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/RapidTrajectoryGenerator.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/RapidTrajectoryGenerator.cpp$(PreprocessSuffix): RapidTrajectoryGenerator.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/RapidTrajectoryGenerator.cpp$(PreprocessSuffix) RapidTrajectoryGenerator.cpp

$(IntermediateDirectory)/Demo.cpp$(ObjectSuffix): Demo.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/Demo.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/Demo.cpp$(DependSuffix) -MM Demo.cpp
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/chh/RapidQuadrocopterTrajectories/C++/Demo.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/Demo.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/Demo.cpp$(PreprocessSuffix): Demo.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/Demo.cpp$(PreprocessSuffix) Demo.cpp


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r $(ConfigurationName)/


