äū
Øų
^
AssignVariableOp
resource
value"dtype"
dtypetype"
validate_shapebool( 

BiasAdd

value"T	
bias"T
output"T""
Ttype:
2	"-
data_formatstringNHWC:
NHWCNCHW
8
Const
output"dtype"
valuetensor"
dtypetype
$
DisableCopyOnRead
resource
.
Identity

input"T
output"T"	
Ttype
u
MatMul
a"T
b"T
product"T"
transpose_abool( "
transpose_bbool( "
Ttype:
2	

MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool("
allow_missing_filesbool( 

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetype
E
Relu
features"T
activations"T"
Ttype:
2	
[
Reshape
tensor"T
shape"Tshape
output"T"	
Ttype"
Tshapetype0:
2	
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0
?
Select
	condition

t"T
e"T
output"T"	
Ttype
d
Shape

input"T&
output"out_typeķout_type"	
Ttype"
out_typetype0:
2	
H
ShardedFilename
basename	
shard

num_shards
filename
0
Sigmoid
x"T
y"T"
Ttype:

2
Į
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring Ø
@
StaticRegexFullMatch	
input

output
"
patternstring
÷
StridedSlice

input"T
begin"Index
end"Index
strides"Index
output"T"	
Ttype"
Indextype:
2	"

begin_maskint "
end_maskint "
ellipsis_maskint "
new_axis_maskint "
shrink_axis_maskint 
L

StringJoin
inputs*N

output"

Nint("
	separatorstring 
°
VarHandleOp
resource"
	containerstring "
shared_namestring "

debug_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 "serve*2.15.02v2.15.0-rc1-8-g6887368d6d48¦É
v
countVarHandleOp*
_output_shapes
: *

debug_namecount/*
dtype0*
shape: *
shared_namecount
W
count/Read/ReadVariableOpReadVariableOpcount*
_output_shapes
: *
dtype0
v
totalVarHandleOp*
_output_shapes
: *

debug_nametotal/*
dtype0*
shape: *
shared_nametotal
W
total/Read/ReadVariableOpReadVariableOptotal*
_output_shapes
: *
dtype0
¦
Adam/v/dense_1/biasVarHandleOp*
_output_shapes
: *$

debug_nameAdam/v/dense_1/bias/*
dtype0*
shape:*$
shared_nameAdam/v/dense_1/bias
y
'Adam/v/dense_1/bias/Read/ReadVariableOpReadVariableOpAdam/v/dense_1/bias*
_output_shapes

:*
dtype0
¦
Adam/m/dense_1/biasVarHandleOp*
_output_shapes
: *$

debug_nameAdam/m/dense_1/bias/*
dtype0*
shape:*$
shared_nameAdam/m/dense_1/bias
y
'Adam/m/dense_1/bias/Read/ReadVariableOpReadVariableOpAdam/m/dense_1/bias*
_output_shapes

:*
dtype0
±
Adam/v/dense_1/kernelVarHandleOp*
_output_shapes
: *&

debug_nameAdam/v/dense_1/kernel/*
dtype0*
shape:*&
shared_nameAdam/v/dense_1/kernel

)Adam/v/dense_1/kernel/Read/ReadVariableOpReadVariableOpAdam/v/dense_1/kernel*!
_output_shapes
:*
dtype0
±
Adam/m/dense_1/kernelVarHandleOp*
_output_shapes
: *&

debug_nameAdam/m/dense_1/kernel/*
dtype0*
shape:*&
shared_nameAdam/m/dense_1/kernel

)Adam/m/dense_1/kernel/Read/ReadVariableOpReadVariableOpAdam/m/dense_1/kernel*!
_output_shapes
:*
dtype0

Adam/v/dense/biasVarHandleOp*
_output_shapes
: *"

debug_nameAdam/v/dense/bias/*
dtype0*
shape:*"
shared_nameAdam/v/dense/bias
t
%Adam/v/dense/bias/Read/ReadVariableOpReadVariableOpAdam/v/dense/bias*
_output_shapes	
:*
dtype0

Adam/m/dense/biasVarHandleOp*
_output_shapes
: *"

debug_nameAdam/m/dense/bias/*
dtype0*
shape:*"
shared_nameAdam/m/dense/bias
t
%Adam/m/dense/bias/Read/ReadVariableOpReadVariableOpAdam/m/dense/bias*
_output_shapes	
:*
dtype0
«
Adam/v/dense/kernelVarHandleOp*
_output_shapes
: *$

debug_nameAdam/v/dense/kernel/*
dtype0*
shape:*$
shared_nameAdam/v/dense/kernel
~
'Adam/v/dense/kernel/Read/ReadVariableOpReadVariableOpAdam/v/dense/kernel*!
_output_shapes
:*
dtype0
«
Adam/m/dense/kernelVarHandleOp*
_output_shapes
: *$

debug_nameAdam/m/dense/kernel/*
dtype0*
shape:*$
shared_nameAdam/m/dense/kernel
~
'Adam/m/dense/kernel/Read/ReadVariableOpReadVariableOpAdam/m/dense/kernel*!
_output_shapes
:*
dtype0

learning_rateVarHandleOp*
_output_shapes
: *

debug_namelearning_rate/*
dtype0*
shape: *
shared_namelearning_rate
g
!learning_rate/Read/ReadVariableOpReadVariableOplearning_rate*
_output_shapes
: *
dtype0

	iterationVarHandleOp*
_output_shapes
: *

debug_name
iteration/*
dtype0	*
shape: *
shared_name	iteration
_
iteration/Read/ReadVariableOpReadVariableOp	iteration*
_output_shapes
: *
dtype0	

dense_1/biasVarHandleOp*
_output_shapes
: *

debug_namedense_1/bias/*
dtype0*
shape:*
shared_namedense_1/bias
k
 dense_1/bias/Read/ReadVariableOpReadVariableOpdense_1/bias*
_output_shapes

:*
dtype0

dense_1/kernelVarHandleOp*
_output_shapes
: *

debug_namedense_1/kernel/*
dtype0*
shape:*
shared_namedense_1/kernel
t
"dense_1/kernel/Read/ReadVariableOpReadVariableOpdense_1/kernel*!
_output_shapes
:*
dtype0


dense/biasVarHandleOp*
_output_shapes
: *

debug_namedense/bias/*
dtype0*
shape:*
shared_name
dense/bias
f
dense/bias/Read/ReadVariableOpReadVariableOp
dense/bias*
_output_shapes	
:*
dtype0

dense/kernelVarHandleOp*
_output_shapes
: *

debug_namedense/kernel/*
dtype0*
shape:*
shared_namedense/kernel
p
 dense/kernel/Read/ReadVariableOpReadVariableOpdense/kernel*!
_output_shapes
:*
dtype0

serving_default_input_1Placeholder*1
_output_shapes
:’’’’’’’’’*
dtype0*&
shape:’’’’’’’’’
ś
StatefulPartitionedCallStatefulPartitionedCallserving_default_input_1dense/kernel
dense/biasdense_1/kerneldense_1/bias*
Tin	
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’*&
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *+
f&R$
"__inference_signature_wrapper_4332

NoOpNoOp
/
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*Ć.
value¹.B¶. BÆ.
ę
	variables
trainable_variables
regularization_losses
	keras_api
__call__
*&call_and_return_all_conditional_losses
_default_save_signature
encoder
	decoder

	optimizer

signatures*
 
0
1
2
3*
 
0
1
2
3*
* 
°
non_trainable_variables

layers
metrics
layer_regularization_losses
layer_metrics
	variables
trainable_variables
regularization_losses
__call__
_default_save_signature
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses*

trace_0
trace_1* 

trace_0
trace_1* 
* 
Ä
layer-0
layer_with_weights-0
layer-1
	variables
trainable_variables
regularization_losses
	keras_api
__call__
* &call_and_return_all_conditional_losses*
Ä
!layer_with_weights-0
!layer-0
"layer-1
#	variables
$trainable_variables
%regularization_losses
&	keras_api
'__call__
*(&call_and_return_all_conditional_losses*

)
_variables
*_iterations
+_learning_rate
,_index_dict
-
_momentums
._velocities
/_update_step_xla*

0serving_default* 
LF
VARIABLE_VALUEdense/kernel&variables/0/.ATTRIBUTES/VARIABLE_VALUE*
JD
VARIABLE_VALUE
dense/bias&variables/1/.ATTRIBUTES/VARIABLE_VALUE*
NH
VARIABLE_VALUEdense_1/kernel&variables/2/.ATTRIBUTES/VARIABLE_VALUE*
LF
VARIABLE_VALUEdense_1/bias&variables/3/.ATTRIBUTES/VARIABLE_VALUE*
* 

0
	1*

10*
* 
* 
* 
* 
* 
* 

2	variables
3trainable_variables
4regularization_losses
5	keras_api
6__call__
*7&call_and_return_all_conditional_losses* 
¦
8	variables
9trainable_variables
:regularization_losses
;	keras_api
<__call__
*=&call_and_return_all_conditional_losses

kernel
bias*

0
1*

0
1*
* 

>non_trainable_variables

?layers
@metrics
Alayer_regularization_losses
Blayer_metrics
	variables
trainable_variables
regularization_losses
__call__
* &call_and_return_all_conditional_losses
& "call_and_return_conditional_losses*

Ctrace_0
Dtrace_1* 

Etrace_0
Ftrace_1* 
¦
G	variables
Htrainable_variables
Iregularization_losses
J	keras_api
K__call__
*L&call_and_return_all_conditional_losses

kernel
bias*

M	variables
Ntrainable_variables
Oregularization_losses
P	keras_api
Q__call__
*R&call_and_return_all_conditional_losses* 

0
1*

0
1*
* 

Snon_trainable_variables

Tlayers
Umetrics
Vlayer_regularization_losses
Wlayer_metrics
#	variables
$trainable_variables
%regularization_losses
'__call__
*(&call_and_return_all_conditional_losses
&("call_and_return_conditional_losses*

Xtrace_0
Ytrace_1* 

Ztrace_0
[trace_1* 
C
*0
\1
]2
^3
_4
`5
a6
b7
c8*
SM
VARIABLE_VALUE	iteration0optimizer/_iterations/.ATTRIBUTES/VARIABLE_VALUE*
ZT
VARIABLE_VALUElearning_rate3optimizer/_learning_rate/.ATTRIBUTES/VARIABLE_VALUE*
* 
 
\0
^1
`2
b3*
 
]0
_1
a2
c3*
* 
* 
8
d	variables
e	keras_api
	ftotal
	gcount*
* 
* 
* 

hnon_trainable_variables

ilayers
jmetrics
klayer_regularization_losses
llayer_metrics
2	variables
3trainable_variables
4regularization_losses
6__call__
*7&call_and_return_all_conditional_losses
&7"call_and_return_conditional_losses* 

mtrace_0* 

ntrace_0* 

0
1*

0
1*
* 

onon_trainable_variables

players
qmetrics
rlayer_regularization_losses
slayer_metrics
8	variables
9trainable_variables
:regularization_losses
<__call__
*=&call_and_return_all_conditional_losses
&="call_and_return_conditional_losses*

ttrace_0* 

utrace_0* 
* 

0
1*
* 
* 
* 
* 
* 
* 
* 

0
1*

0
1*
* 

vnon_trainable_variables

wlayers
xmetrics
ylayer_regularization_losses
zlayer_metrics
G	variables
Htrainable_variables
Iregularization_losses
K__call__
*L&call_and_return_all_conditional_losses
&L"call_and_return_conditional_losses*

{trace_0* 

|trace_0* 
* 
* 
* 

}non_trainable_variables

~layers
metrics
 layer_regularization_losses
layer_metrics
M	variables
Ntrainable_variables
Oregularization_losses
Q__call__
*R&call_and_return_all_conditional_losses
&R"call_and_return_conditional_losses* 

trace_0* 

trace_0* 
* 

!0
"1*
* 
* 
* 
* 
* 
* 
* 
^X
VARIABLE_VALUEAdam/m/dense/kernel1optimizer/_variables/1/.ATTRIBUTES/VARIABLE_VALUE*
^X
VARIABLE_VALUEAdam/v/dense/kernel1optimizer/_variables/2/.ATTRIBUTES/VARIABLE_VALUE*
\V
VARIABLE_VALUEAdam/m/dense/bias1optimizer/_variables/3/.ATTRIBUTES/VARIABLE_VALUE*
\V
VARIABLE_VALUEAdam/v/dense/bias1optimizer/_variables/4/.ATTRIBUTES/VARIABLE_VALUE*
`Z
VARIABLE_VALUEAdam/m/dense_1/kernel1optimizer/_variables/5/.ATTRIBUTES/VARIABLE_VALUE*
`Z
VARIABLE_VALUEAdam/v/dense_1/kernel1optimizer/_variables/6/.ATTRIBUTES/VARIABLE_VALUE*
^X
VARIABLE_VALUEAdam/m/dense_1/bias1optimizer/_variables/7/.ATTRIBUTES/VARIABLE_VALUE*
^X
VARIABLE_VALUEAdam/v/dense_1/bias1optimizer/_variables/8/.ATTRIBUTES/VARIABLE_VALUE*

f0
g1*

d	variables*
SM
VARIABLE_VALUEtotal4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUE*
SM
VARIABLE_VALUEcount4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUE*
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
* 
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
±
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filenamedense/kernel
dense/biasdense_1/kerneldense_1/bias	iterationlearning_rateAdam/m/dense/kernelAdam/v/dense/kernelAdam/m/dense/biasAdam/v/dense/biasAdam/m/dense_1/kernelAdam/v/dense_1/kernelAdam/m/dense_1/biasAdam/v/dense_1/biastotalcountConst*
Tin
2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *&
f!R
__inference__traced_save_4520
¬
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenamedense/kernel
dense/biasdense_1/kerneldense_1/bias	iterationlearning_rateAdam/m/dense/kernelAdam/v/dense/kernelAdam/m/dense/biasAdam/v/dense/biasAdam/m/dense_1/kernelAdam/v/dense_1/kernelAdam/m/dense_1/biasAdam/v/dense_1/biastotalcount*
Tin
2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *)
f$R"
 __inference__traced_restore_4577āź
É
]
A__inference_flatten_layer_call_and_return_conditional_losses_4124

inputs
identityV
ConstConst*
_output_shapes
:*
dtype0*
valueB"’’’’ Ą  ^
ReshapeReshapeinputsConst:output:0*
T0*)
_output_shapes
:’’’’’’’’’Z
IdentityIdentityReshape:output:0*
T0*)
_output_shapes
:’’’’’’’’’"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*0
_input_shapes
:’’’’’’’’’:Y U
1
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
Ū

÷
A__inference_dense_1_layer_call_and_return_conditional_losses_4198

inputs3
matmul_readvariableop_resource:/
biasadd_readvariableop_resource:

identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOpw
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*!
_output_shapes
:*
dtype0k
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*)
_output_shapes
:’’’’’’’’’t
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes

:*
dtype0x
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*)
_output_shapes
:’’’’’’’’’X
SigmoidSigmoidBiasAdd:output:0*
T0*)
_output_shapes
:’’’’’’’’’\
IdentityIdentitySigmoid:y:0^NoOp*
T0*)
_output_shapes
:’’’’’’’’’S
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:’’’’’’’’’: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:($
"
_user_specified_name
resource:($
"
_user_specified_name
resource:P L
(
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
Ą	
×
*__inference_autoencoder_layer_call_fn_4316
input_1
unknown:
	unknown_0:	
	unknown_1:
	unknown_2:

identity¢StatefulPartitionedCall’
StatefulPartitionedCallStatefulPartitionedCallinput_1unknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’*&
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_autoencoder_layer_call_and_return_conditional_losses_4290y
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*8
_input_shapes'
%:’’’’’’’’’: : : : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4312:$ 

_user_specified_name4310:$ 

_user_specified_name4308:$ 

_user_specified_name4306:Z V
1
_output_shapes
:’’’’’’’’’
!
_user_specified_name	input_1
ą
]
A__inference_reshape_layer_call_and_return_conditional_losses_4402

inputs
identityI
ShapeShapeinputs*
T0*
_output_shapes
::ķĻ]
strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: _
strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:_
strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:Ń
strided_sliceStridedSliceShape:output:0strided_slice/stack:output:0strided_slice/stack_1:output:0strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_maskR
Reshape/shape/1Const*
_output_shapes
: *
dtype0*
value
B :R
Reshape/shape/2Const*
_output_shapes
: *
dtype0*
value
B :Q
Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :©
Reshape/shapePackstrided_slice:output:0Reshape/shape/1:output:0Reshape/shape/2:output:0Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:n
ReshapeReshapeinputsReshape/shape:output:0*
T0*1
_output_shapes
:’’’’’’’’’b
IdentityIdentityReshape:output:0*
T0*1
_output_shapes
:’’’’’’’’’"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*(
_input_shapes
:’’’’’’’’’:Q M
)
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
ź

$__inference_dense_layer_call_fn_4352

inputs
unknown:
	unknown_0:	
identity¢StatefulPartitionedCallÕ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *H
fCRA
?__inference_dense_layer_call_and_return_conditional_losses_4136p
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*(
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*,
_input_shapes
:’’’’’’’’’: : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4348:$ 

_user_specified_name4346:Q M
)
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs

”
)__inference_sequential_layer_call_fn_4171
flatten_input
unknown:
	unknown_0:	
identity¢StatefulPartitionedCallį
StatefulPartitionedCallStatefulPartitionedCallflatten_inputunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *M
fHRF
D__inference_sequential_layer_call_and_return_conditional_losses_4153p
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*(
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*4
_input_shapes#
!:’’’’’’’’’: : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4167:$ 

_user_specified_name4165:` \
1
_output_shapes
:’’’’’’’’’
'
_user_specified_nameflatten_input
	
Ļ
"__inference_signature_wrapper_4332
input_1
unknown:
	unknown_0:	
	unknown_1:
	unknown_2:

identity¢StatefulPartitionedCallŁ
StatefulPartitionedCallStatefulPartitionedCallinput_1unknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’*&
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *(
f#R!
__inference__wrapped_model_4116y
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*8
_input_shapes'
%:’’’’’’’’’: : : : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4328:$ 

_user_specified_name4326:$ 

_user_specified_name4324:$ 

_user_specified_name4322:Z V
1
_output_shapes
:’’’’’’’’’
!
_user_specified_name	input_1
ō
Ā
__inference__traced_save_4520
file_prefix8
#read_disablecopyonread_dense_kernel:2
#read_1_disablecopyonread_dense_bias:	<
'read_2_disablecopyonread_dense_1_kernel:5
%read_3_disablecopyonread_dense_1_bias:
,
"read_4_disablecopyonread_iteration:	 0
&read_5_disablecopyonread_learning_rate: A
,read_6_disablecopyonread_adam_m_dense_kernel:A
,read_7_disablecopyonread_adam_v_dense_kernel:9
*read_8_disablecopyonread_adam_m_dense_bias:	9
*read_9_disablecopyonread_adam_v_dense_bias:	D
/read_10_disablecopyonread_adam_m_dense_1_kernel:D
/read_11_disablecopyonread_adam_v_dense_1_kernel:=
-read_12_disablecopyonread_adam_m_dense_1_bias:
=
-read_13_disablecopyonread_adam_v_dense_1_bias:
)
read_14_disablecopyonread_total: )
read_15_disablecopyonread_count: 
savev2_const
identity_33¢MergeV2Checkpoints¢Read/DisableCopyOnRead¢Read/ReadVariableOp¢Read_1/DisableCopyOnRead¢Read_1/ReadVariableOp¢Read_10/DisableCopyOnRead¢Read_10/ReadVariableOp¢Read_11/DisableCopyOnRead¢Read_11/ReadVariableOp¢Read_12/DisableCopyOnRead¢Read_12/ReadVariableOp¢Read_13/DisableCopyOnRead¢Read_13/ReadVariableOp¢Read_14/DisableCopyOnRead¢Read_14/ReadVariableOp¢Read_15/DisableCopyOnRead¢Read_15/ReadVariableOp¢Read_2/DisableCopyOnRead¢Read_2/ReadVariableOp¢Read_3/DisableCopyOnRead¢Read_3/ReadVariableOp¢Read_4/DisableCopyOnRead¢Read_4/ReadVariableOp¢Read_5/DisableCopyOnRead¢Read_5/ReadVariableOp¢Read_6/DisableCopyOnRead¢Read_6/ReadVariableOp¢Read_7/DisableCopyOnRead¢Read_7/ReadVariableOp¢Read_8/DisableCopyOnRead¢Read_8/ReadVariableOp¢Read_9/DisableCopyOnRead¢Read_9/ReadVariableOpw
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*Z
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.parta
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: f

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: L

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :f
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : 
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: u
Read/DisableCopyOnReadDisableCopyOnRead#read_disablecopyonread_dense_kernel"/device:CPU:0*
_output_shapes
 ¢
Read/ReadVariableOpReadVariableOp#read_disablecopyonread_dense_kernel^Read/DisableCopyOnRead"/device:CPU:0*!
_output_shapes
:*
dtype0l
IdentityIdentityRead/ReadVariableOp:value:0"/device:CPU:0*
T0*!
_output_shapes
:d

Identity_1IdentityIdentity:output:0"/device:CPU:0*
T0*!
_output_shapes
:w
Read_1/DisableCopyOnReadDisableCopyOnRead#read_1_disablecopyonread_dense_bias"/device:CPU:0*
_output_shapes
  
Read_1/ReadVariableOpReadVariableOp#read_1_disablecopyonread_dense_bias^Read_1/DisableCopyOnRead"/device:CPU:0*
_output_shapes	
:*
dtype0j

Identity_2IdentityRead_1/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes	
:`

Identity_3IdentityIdentity_2:output:0"/device:CPU:0*
T0*
_output_shapes	
:{
Read_2/DisableCopyOnReadDisableCopyOnRead'read_2_disablecopyonread_dense_1_kernel"/device:CPU:0*
_output_shapes
 Ŗ
Read_2/ReadVariableOpReadVariableOp'read_2_disablecopyonread_dense_1_kernel^Read_2/DisableCopyOnRead"/device:CPU:0*!
_output_shapes
:*
dtype0p

Identity_4IdentityRead_2/ReadVariableOp:value:0"/device:CPU:0*
T0*!
_output_shapes
:f

Identity_5IdentityIdentity_4:output:0"/device:CPU:0*
T0*!
_output_shapes
:y
Read_3/DisableCopyOnReadDisableCopyOnRead%read_3_disablecopyonread_dense_1_bias"/device:CPU:0*
_output_shapes
 £
Read_3/ReadVariableOpReadVariableOp%read_3_disablecopyonread_dense_1_bias^Read_3/DisableCopyOnRead"/device:CPU:0*
_output_shapes

:*
dtype0k

Identity_6IdentityRead_3/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes

:a

Identity_7IdentityIdentity_6:output:0"/device:CPU:0*
T0*
_output_shapes

:v
Read_4/DisableCopyOnReadDisableCopyOnRead"read_4_disablecopyonread_iteration"/device:CPU:0*
_output_shapes
 
Read_4/ReadVariableOpReadVariableOp"read_4_disablecopyonread_iteration^Read_4/DisableCopyOnRead"/device:CPU:0*
_output_shapes
: *
dtype0	e

Identity_8IdentityRead_4/ReadVariableOp:value:0"/device:CPU:0*
T0	*
_output_shapes
: [

Identity_9IdentityIdentity_8:output:0"/device:CPU:0*
T0	*
_output_shapes
: z
Read_5/DisableCopyOnReadDisableCopyOnRead&read_5_disablecopyonread_learning_rate"/device:CPU:0*
_output_shapes
 
Read_5/ReadVariableOpReadVariableOp&read_5_disablecopyonread_learning_rate^Read_5/DisableCopyOnRead"/device:CPU:0*
_output_shapes
: *
dtype0f
Identity_10IdentityRead_5/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes
: ]
Identity_11IdentityIdentity_10:output:0"/device:CPU:0*
T0*
_output_shapes
: 
Read_6/DisableCopyOnReadDisableCopyOnRead,read_6_disablecopyonread_adam_m_dense_kernel"/device:CPU:0*
_output_shapes
 Æ
Read_6/ReadVariableOpReadVariableOp,read_6_disablecopyonread_adam_m_dense_kernel^Read_6/DisableCopyOnRead"/device:CPU:0*!
_output_shapes
:*
dtype0q
Identity_12IdentityRead_6/ReadVariableOp:value:0"/device:CPU:0*
T0*!
_output_shapes
:h
Identity_13IdentityIdentity_12:output:0"/device:CPU:0*
T0*!
_output_shapes
:
Read_7/DisableCopyOnReadDisableCopyOnRead,read_7_disablecopyonread_adam_v_dense_kernel"/device:CPU:0*
_output_shapes
 Æ
Read_7/ReadVariableOpReadVariableOp,read_7_disablecopyonread_adam_v_dense_kernel^Read_7/DisableCopyOnRead"/device:CPU:0*!
_output_shapes
:*
dtype0q
Identity_14IdentityRead_7/ReadVariableOp:value:0"/device:CPU:0*
T0*!
_output_shapes
:h
Identity_15IdentityIdentity_14:output:0"/device:CPU:0*
T0*!
_output_shapes
:~
Read_8/DisableCopyOnReadDisableCopyOnRead*read_8_disablecopyonread_adam_m_dense_bias"/device:CPU:0*
_output_shapes
 §
Read_8/ReadVariableOpReadVariableOp*read_8_disablecopyonread_adam_m_dense_bias^Read_8/DisableCopyOnRead"/device:CPU:0*
_output_shapes	
:*
dtype0k
Identity_16IdentityRead_8/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes	
:b
Identity_17IdentityIdentity_16:output:0"/device:CPU:0*
T0*
_output_shapes	
:~
Read_9/DisableCopyOnReadDisableCopyOnRead*read_9_disablecopyonread_adam_v_dense_bias"/device:CPU:0*
_output_shapes
 §
Read_9/ReadVariableOpReadVariableOp*read_9_disablecopyonread_adam_v_dense_bias^Read_9/DisableCopyOnRead"/device:CPU:0*
_output_shapes	
:*
dtype0k
Identity_18IdentityRead_9/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes	
:b
Identity_19IdentityIdentity_18:output:0"/device:CPU:0*
T0*
_output_shapes	
:
Read_10/DisableCopyOnReadDisableCopyOnRead/read_10_disablecopyonread_adam_m_dense_1_kernel"/device:CPU:0*
_output_shapes
 “
Read_10/ReadVariableOpReadVariableOp/read_10_disablecopyonread_adam_m_dense_1_kernel^Read_10/DisableCopyOnRead"/device:CPU:0*!
_output_shapes
:*
dtype0r
Identity_20IdentityRead_10/ReadVariableOp:value:0"/device:CPU:0*
T0*!
_output_shapes
:h
Identity_21IdentityIdentity_20:output:0"/device:CPU:0*
T0*!
_output_shapes
:
Read_11/DisableCopyOnReadDisableCopyOnRead/read_11_disablecopyonread_adam_v_dense_1_kernel"/device:CPU:0*
_output_shapes
 “
Read_11/ReadVariableOpReadVariableOp/read_11_disablecopyonread_adam_v_dense_1_kernel^Read_11/DisableCopyOnRead"/device:CPU:0*!
_output_shapes
:*
dtype0r
Identity_22IdentityRead_11/ReadVariableOp:value:0"/device:CPU:0*
T0*!
_output_shapes
:h
Identity_23IdentityIdentity_22:output:0"/device:CPU:0*
T0*!
_output_shapes
:
Read_12/DisableCopyOnReadDisableCopyOnRead-read_12_disablecopyonread_adam_m_dense_1_bias"/device:CPU:0*
_output_shapes
 ­
Read_12/ReadVariableOpReadVariableOp-read_12_disablecopyonread_adam_m_dense_1_bias^Read_12/DisableCopyOnRead"/device:CPU:0*
_output_shapes

:*
dtype0m
Identity_24IdentityRead_12/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes

:c
Identity_25IdentityIdentity_24:output:0"/device:CPU:0*
T0*
_output_shapes

:
Read_13/DisableCopyOnReadDisableCopyOnRead-read_13_disablecopyonread_adam_v_dense_1_bias"/device:CPU:0*
_output_shapes
 ­
Read_13/ReadVariableOpReadVariableOp-read_13_disablecopyonread_adam_v_dense_1_bias^Read_13/DisableCopyOnRead"/device:CPU:0*
_output_shapes

:*
dtype0m
Identity_26IdentityRead_13/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes

:c
Identity_27IdentityIdentity_26:output:0"/device:CPU:0*
T0*
_output_shapes

:t
Read_14/DisableCopyOnReadDisableCopyOnReadread_14_disablecopyonread_total"/device:CPU:0*
_output_shapes
 
Read_14/ReadVariableOpReadVariableOpread_14_disablecopyonread_total^Read_14/DisableCopyOnRead"/device:CPU:0*
_output_shapes
: *
dtype0g
Identity_28IdentityRead_14/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes
: ]
Identity_29IdentityIdentity_28:output:0"/device:CPU:0*
T0*
_output_shapes
: t
Read_15/DisableCopyOnReadDisableCopyOnReadread_15_disablecopyonread_count"/device:CPU:0*
_output_shapes
 
Read_15/ReadVariableOpReadVariableOpread_15_disablecopyonread_count^Read_15/DisableCopyOnRead"/device:CPU:0*
_output_shapes
: *
dtype0g
Identity_30IdentityRead_15/ReadVariableOp:value:0"/device:CPU:0*
T0*
_output_shapes
: ]
Identity_31IdentityIdentity_30:output:0"/device:CPU:0*
T0*
_output_shapes
: 
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*¾
value“B±B&variables/0/.ATTRIBUTES/VARIABLE_VALUEB&variables/1/.ATTRIBUTES/VARIABLE_VALUEB&variables/2/.ATTRIBUTES/VARIABLE_VALUEB&variables/3/.ATTRIBUTES/VARIABLE_VALUEB0optimizer/_iterations/.ATTRIBUTES/VARIABLE_VALUEB3optimizer/_learning_rate/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/1/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/2/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/3/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/4/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/5/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/6/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/7/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/8/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*5
value,B*B B B B B B B B B B B B B B B B B Ć
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:0Identity_1:output:0Identity_3:output:0Identity_5:output:0Identity_7:output:0Identity_9:output:0Identity_11:output:0Identity_13:output:0Identity_15:output:0Identity_17:output:0Identity_19:output:0Identity_21:output:0Identity_23:output:0Identity_25:output:0Identity_27:output:0Identity_29:output:0Identity_31:output:0savev2_const"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtypes
2	
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:³
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 i
Identity_32Identityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: U
Identity_33IdentityIdentity_32:output:0^NoOp*
T0*
_output_shapes
: ļ
NoOpNoOp^MergeV2Checkpoints^Read/DisableCopyOnRead^Read/ReadVariableOp^Read_1/DisableCopyOnRead^Read_1/ReadVariableOp^Read_10/DisableCopyOnRead^Read_10/ReadVariableOp^Read_11/DisableCopyOnRead^Read_11/ReadVariableOp^Read_12/DisableCopyOnRead^Read_12/ReadVariableOp^Read_13/DisableCopyOnRead^Read_13/ReadVariableOp^Read_14/DisableCopyOnRead^Read_14/ReadVariableOp^Read_15/DisableCopyOnRead^Read_15/ReadVariableOp^Read_2/DisableCopyOnRead^Read_2/ReadVariableOp^Read_3/DisableCopyOnRead^Read_3/ReadVariableOp^Read_4/DisableCopyOnRead^Read_4/ReadVariableOp^Read_5/DisableCopyOnRead^Read_5/ReadVariableOp^Read_6/DisableCopyOnRead^Read_6/ReadVariableOp^Read_7/DisableCopyOnRead^Read_7/ReadVariableOp^Read_8/DisableCopyOnRead^Read_8/ReadVariableOp^Read_9/DisableCopyOnRead^Read_9/ReadVariableOp*
_output_shapes
 "#
identity_33Identity_33:output:0*(
_construction_contextkEagerRuntime*7
_input_shapes&
$: : : : : : : : : : : : : : : : : : 2(
MergeV2CheckpointsMergeV2Checkpoints20
Read/DisableCopyOnReadRead/DisableCopyOnRead2*
Read/ReadVariableOpRead/ReadVariableOp24
Read_1/DisableCopyOnReadRead_1/DisableCopyOnRead2.
Read_1/ReadVariableOpRead_1/ReadVariableOp26
Read_10/DisableCopyOnReadRead_10/DisableCopyOnRead20
Read_10/ReadVariableOpRead_10/ReadVariableOp26
Read_11/DisableCopyOnReadRead_11/DisableCopyOnRead20
Read_11/ReadVariableOpRead_11/ReadVariableOp26
Read_12/DisableCopyOnReadRead_12/DisableCopyOnRead20
Read_12/ReadVariableOpRead_12/ReadVariableOp26
Read_13/DisableCopyOnReadRead_13/DisableCopyOnRead20
Read_13/ReadVariableOpRead_13/ReadVariableOp26
Read_14/DisableCopyOnReadRead_14/DisableCopyOnRead20
Read_14/ReadVariableOpRead_14/ReadVariableOp26
Read_15/DisableCopyOnReadRead_15/DisableCopyOnRead20
Read_15/ReadVariableOpRead_15/ReadVariableOp24
Read_2/DisableCopyOnReadRead_2/DisableCopyOnRead2.
Read_2/ReadVariableOpRead_2/ReadVariableOp24
Read_3/DisableCopyOnReadRead_3/DisableCopyOnRead2.
Read_3/ReadVariableOpRead_3/ReadVariableOp24
Read_4/DisableCopyOnReadRead_4/DisableCopyOnRead2.
Read_4/ReadVariableOpRead_4/ReadVariableOp24
Read_5/DisableCopyOnReadRead_5/DisableCopyOnRead2.
Read_5/ReadVariableOpRead_5/ReadVariableOp24
Read_6/DisableCopyOnReadRead_6/DisableCopyOnRead2.
Read_6/ReadVariableOpRead_6/ReadVariableOp24
Read_7/DisableCopyOnReadRead_7/DisableCopyOnRead2.
Read_7/ReadVariableOpRead_7/ReadVariableOp24
Read_8/DisableCopyOnReadRead_8/DisableCopyOnRead2.
Read_8/ReadVariableOpRead_8/ReadVariableOp24
Read_9/DisableCopyOnReadRead_9/DisableCopyOnRead2.
Read_9/ReadVariableOpRead_9/ReadVariableOp:=9

_output_shapes
: 

_user_specified_nameConst:%!

_user_specified_namecount:%!

_user_specified_nametotal:3/
-
_user_specified_nameAdam/v/dense_1/bias:3/
-
_user_specified_nameAdam/m/dense_1/bias:51
/
_user_specified_nameAdam/v/dense_1/kernel:51
/
_user_specified_nameAdam/m/dense_1/kernel:1
-
+
_user_specified_nameAdam/v/dense/bias:1	-
+
_user_specified_nameAdam/m/dense/bias:3/
-
_user_specified_nameAdam/v/dense/kernel:3/
-
_user_specified_nameAdam/m/dense/kernel:-)
'
_user_specified_namelearning_rate:)%
#
_user_specified_name	iteration:,(
&
_user_specified_namedense_1/bias:.*
(
_user_specified_namedense_1/kernel:*&
$
_user_specified_name
dense/bias:,(
&
_user_specified_namedense/kernel:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
š
Ā
E__inference_autoencoder_layer_call_and_return_conditional_losses_4290
input_1$
sequential_4279:
sequential_4281:	&
sequential_1_4284:!
sequential_1_4286:

identity¢"sequential/StatefulPartitionedCall¢$sequential_1/StatefulPartitionedCallō
"sequential/StatefulPartitionedCallStatefulPartitionedCallinput_1sequential_4279sequential_4281*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *M
fHRF
D__inference_sequential_layer_call_and_return_conditional_losses_4153©
$sequential_1/StatefulPartitionedCallStatefulPartitionedCall+sequential/StatefulPartitionedCall:output:0sequential_1_4284sequential_1_4286*
Tin
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_sequential_1_layer_call_and_return_conditional_losses_4230
IdentityIdentity-sequential_1/StatefulPartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’n
NoOpNoOp#^sequential/StatefulPartitionedCall%^sequential_1/StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*8
_input_shapes'
%:’’’’’’’’’: : : : 2H
"sequential/StatefulPartitionedCall"sequential/StatefulPartitionedCall2L
$sequential_1/StatefulPartitionedCall$sequential_1/StatefulPartitionedCall:$ 

_user_specified_name4286:$ 

_user_specified_name4284:$ 

_user_specified_name4281:$ 

_user_specified_name4279:Z V
1
_output_shapes
:’’’’’’’’’
!
_user_specified_name	input_1
Æ
B
&__inference_flatten_layer_call_fn_4337

inputs
identity®
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *)
_output_shapes
:’’’’’’’’’* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_flatten_layer_call_and_return_conditional_losses_4124b
IdentityIdentityPartitionedCall:output:0*
T0*)
_output_shapes
:’’’’’’’’’"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*0
_input_shapes
:’’’’’’’’’:Y U
1
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
Æ
B
&__inference_reshape_layer_call_fn_4388

inputs
identity¶
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_reshape_layer_call_and_return_conditional_losses_4217j
IdentityIdentityPartitionedCall:output:0*
T0*1
_output_shapes
:’’’’’’’’’"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*(
_input_shapes
:’’’’’’’’’:Q M
)
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
Ē
Ļ
F__inference_sequential_1_layer_call_and_return_conditional_losses_4230
dense_1_input!
dense_1_4223:
dense_1_4225:

identity¢dense_1/StatefulPartitionedCallļ
dense_1/StatefulPartitionedCallStatefulPartitionedCalldense_1_inputdense_1_4223dense_1_4225*
Tin
2*
Tout
2*
_collective_manager_ids
 *)
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_dense_1_layer_call_and_return_conditional_losses_4198ą
reshape/PartitionedCallPartitionedCall(dense_1/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_reshape_layer_call_and_return_conditional_losses_4217y
IdentityIdentity reshape/PartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’D
NoOpNoOp ^dense_1/StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:’’’’’’’’’: : 2B
dense_1/StatefulPartitionedCalldense_1/StatefulPartitionedCall:$ 

_user_specified_name4225:$ 

_user_specified_name4223:W S
(
_output_shapes
:’’’’’’’’’
'
_user_specified_namedense_1_input
ļ

&__inference_dense_1_layer_call_fn_4372

inputs
unknown:
	unknown_0:

identity¢StatefulPartitionedCallŲ
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *)
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_dense_1_layer_call_and_return_conditional_losses_4198q
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*)
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:’’’’’’’’’: : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4368:$ 

_user_specified_name4366:P L
(
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
ą
]
A__inference_reshape_layer_call_and_return_conditional_losses_4217

inputs
identityI
ShapeShapeinputs*
T0*
_output_shapes
::ķĻ]
strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: _
strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:_
strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:Ń
strided_sliceStridedSliceShape:output:0strided_slice/stack:output:0strided_slice/stack_1:output:0strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_maskR
Reshape/shape/1Const*
_output_shapes
: *
dtype0*
value
B :R
Reshape/shape/2Const*
_output_shapes
: *
dtype0*
value
B :Q
Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :©
Reshape/shapePackstrided_slice:output:0Reshape/shape/1:output:0Reshape/shape/2:output:0Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:n
ReshapeReshapeinputsReshape/shape:output:0*
T0*1
_output_shapes
:’’’’’’’’’b
IdentityIdentityReshape:output:0*
T0*1
_output_shapes
:’’’’’’’’’"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*(
_input_shapes
:’’’’’’’’’:Q M
)
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
Ē
Ļ
F__inference_sequential_1_layer_call_and_return_conditional_losses_4220
dense_1_input!
dense_1_4199:
dense_1_4201:

identity¢dense_1/StatefulPartitionedCallļ
dense_1/StatefulPartitionedCallStatefulPartitionedCalldense_1_inputdense_1_4199dense_1_4201*
Tin
2*
Tout
2*
_collective_manager_ids
 *)
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_dense_1_layer_call_and_return_conditional_losses_4198ą
reshape/PartitionedCallPartitionedCall(dense_1/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_reshape_layer_call_and_return_conditional_losses_4217y
IdentityIdentity reshape/PartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’D
NoOpNoOp ^dense_1/StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:’’’’’’’’’: : 2B
dense_1/StatefulPartitionedCalldense_1/StatefulPartitionedCall:$ 

_user_specified_name4201:$ 

_user_specified_name4199:W S
(
_output_shapes
:’’’’’’’’’
'
_user_specified_namedense_1_input
Ö

ō
?__inference_dense_layer_call_and_return_conditional_losses_4136

inputs3
matmul_readvariableop_resource:.
biasadd_readvariableop_resource:	
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOpw
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*!
_output_shapes
:*
dtype0j
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:’’’’’’’’’s
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:*
dtype0w
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:’’’’’’’’’Q
ReluReluBiasAdd:output:0*
T0*(
_output_shapes
:’’’’’’’’’b
IdentityIdentityRelu:activations:0^NoOp*
T0*(
_output_shapes
:’’’’’’’’’S
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*,
_input_shapes
:’’’’’’’’’: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:($
"
_user_specified_name
resource:($
"
_user_specified_name
resource:Q M
)
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
Ö

ō
?__inference_dense_layer_call_and_return_conditional_losses_4363

inputs3
matmul_readvariableop_resource:.
biasadd_readvariableop_resource:	
identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOpw
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*!
_output_shapes
:*
dtype0j
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:’’’’’’’’’s
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:*
dtype0w
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:’’’’’’’’’Q
ReluReluBiasAdd:output:0*
T0*(
_output_shapes
:’’’’’’’’’b
IdentityIdentityRelu:activations:0^NoOp*
T0*(
_output_shapes
:’’’’’’’’’S
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*,
_input_shapes
:’’’’’’’’’: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:($
"
_user_specified_name
resource:($
"
_user_specified_name
resource:Q M
)
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs

¤
+__inference_sequential_1_layer_call_fn_4248
dense_1_input
unknown:
	unknown_0:

identity¢StatefulPartitionedCallģ
StatefulPartitionedCallStatefulPartitionedCalldense_1_inputunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_sequential_1_layer_call_and_return_conditional_losses_4230y
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:’’’’’’’’’: : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4244:$ 

_user_specified_name4242:W S
(
_output_shapes
:’’’’’’’’’
'
_user_specified_namedense_1_input
Ē-
ä
__inference__wrapped_model_4116
input_1P
;autoencoder_sequential_dense_matmul_readvariableop_resource:K
<autoencoder_sequential_dense_biasadd_readvariableop_resource:	T
?autoencoder_sequential_1_dense_1_matmul_readvariableop_resource:P
@autoencoder_sequential_1_dense_1_biasadd_readvariableop_resource:

identity¢3autoencoder/sequential/dense/BiasAdd/ReadVariableOp¢2autoencoder/sequential/dense/MatMul/ReadVariableOp¢7autoencoder/sequential_1/dense_1/BiasAdd/ReadVariableOp¢6autoencoder/sequential_1/dense_1/MatMul/ReadVariableOpu
$autoencoder/sequential/flatten/ConstConst*
_output_shapes
:*
dtype0*
valueB"’’’’ Ą  
&autoencoder/sequential/flatten/ReshapeReshapeinput_1-autoencoder/sequential/flatten/Const:output:0*
T0*)
_output_shapes
:’’’’’’’’’±
2autoencoder/sequential/dense/MatMul/ReadVariableOpReadVariableOp;autoencoder_sequential_dense_matmul_readvariableop_resource*!
_output_shapes
:*
dtype0Ķ
#autoencoder/sequential/dense/MatMulMatMul/autoencoder/sequential/flatten/Reshape:output:0:autoencoder/sequential/dense/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:’’’’’’’’’­
3autoencoder/sequential/dense/BiasAdd/ReadVariableOpReadVariableOp<autoencoder_sequential_dense_biasadd_readvariableop_resource*
_output_shapes	
:*
dtype0Ī
$autoencoder/sequential/dense/BiasAddBiasAdd-autoencoder/sequential/dense/MatMul:product:0;autoencoder/sequential/dense/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:’’’’’’’’’
!autoencoder/sequential/dense/ReluRelu-autoencoder/sequential/dense/BiasAdd:output:0*
T0*(
_output_shapes
:’’’’’’’’’¹
6autoencoder/sequential_1/dense_1/MatMul/ReadVariableOpReadVariableOp?autoencoder_sequential_1_dense_1_matmul_readvariableop_resource*!
_output_shapes
:*
dtype0Ö
'autoencoder/sequential_1/dense_1/MatMulMatMul/autoencoder/sequential/dense/Relu:activations:0>autoencoder/sequential_1/dense_1/MatMul/ReadVariableOp:value:0*
T0*)
_output_shapes
:’’’’’’’’’¶
7autoencoder/sequential_1/dense_1/BiasAdd/ReadVariableOpReadVariableOp@autoencoder_sequential_1_dense_1_biasadd_readvariableop_resource*
_output_shapes

:*
dtype0Ū
(autoencoder/sequential_1/dense_1/BiasAddBiasAdd1autoencoder/sequential_1/dense_1/MatMul:product:0?autoencoder/sequential_1/dense_1/BiasAdd/ReadVariableOp:value:0*
T0*)
_output_shapes
:’’’’’’’’’
(autoencoder/sequential_1/dense_1/SigmoidSigmoid1autoencoder/sequential_1/dense_1/BiasAdd:output:0*
T0*)
_output_shapes
:’’’’’’’’’
&autoencoder/sequential_1/reshape/ShapeShape,autoencoder/sequential_1/dense_1/Sigmoid:y:0*
T0*
_output_shapes
::ķĻ~
4autoencoder/sequential_1/reshape/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 
6autoencoder/sequential_1/reshape/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:
6autoencoder/sequential_1/reshape/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:ö
.autoencoder/sequential_1/reshape/strided_sliceStridedSlice/autoencoder/sequential_1/reshape/Shape:output:0=autoencoder/sequential_1/reshape/strided_slice/stack:output:0?autoencoder/sequential_1/reshape/strided_slice/stack_1:output:0?autoencoder/sequential_1/reshape/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
: *
shrink_axis_masks
0autoencoder/sequential_1/reshape/Reshape/shape/1Const*
_output_shapes
: *
dtype0*
value
B :s
0autoencoder/sequential_1/reshape/Reshape/shape/2Const*
_output_shapes
: *
dtype0*
value
B :r
0autoencoder/sequential_1/reshape/Reshape/shape/3Const*
_output_shapes
: *
dtype0*
value	B :Ī
.autoencoder/sequential_1/reshape/Reshape/shapePack7autoencoder/sequential_1/reshape/strided_slice:output:09autoencoder/sequential_1/reshape/Reshape/shape/1:output:09autoencoder/sequential_1/reshape/Reshape/shape/2:output:09autoencoder/sequential_1/reshape/Reshape/shape/3:output:0*
N*
T0*
_output_shapes
:Ö
(autoencoder/sequential_1/reshape/ReshapeReshape,autoencoder/sequential_1/dense_1/Sigmoid:y:07autoencoder/sequential_1/reshape/Reshape/shape:output:0*
T0*1
_output_shapes
:’’’’’’’’’
IdentityIdentity1autoencoder/sequential_1/reshape/Reshape:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’
NoOpNoOp4^autoencoder/sequential/dense/BiasAdd/ReadVariableOp3^autoencoder/sequential/dense/MatMul/ReadVariableOp8^autoencoder/sequential_1/dense_1/BiasAdd/ReadVariableOp7^autoencoder/sequential_1/dense_1/MatMul/ReadVariableOp*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*8
_input_shapes'
%:’’’’’’’’’: : : : 2j
3autoencoder/sequential/dense/BiasAdd/ReadVariableOp3autoencoder/sequential/dense/BiasAdd/ReadVariableOp2h
2autoencoder/sequential/dense/MatMul/ReadVariableOp2autoencoder/sequential/dense/MatMul/ReadVariableOp2r
7autoencoder/sequential_1/dense_1/BiasAdd/ReadVariableOp7autoencoder/sequential_1/dense_1/BiasAdd/ReadVariableOp2p
6autoencoder/sequential_1/dense_1/MatMul/ReadVariableOp6autoencoder/sequential_1/dense_1/MatMul/ReadVariableOp:($
"
_user_specified_name
resource:($
"
_user_specified_name
resource:($
"
_user_specified_name
resource:($
"
_user_specified_name
resource:Z V
1
_output_shapes
:’’’’’’’’’
!
_user_specified_name	input_1

¤
+__inference_sequential_1_layer_call_fn_4239
dense_1_input
unknown:
	unknown_0:

identity¢StatefulPartitionedCallģ
StatefulPartitionedCallStatefulPartitionedCalldense_1_inputunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_sequential_1_layer_call_and_return_conditional_losses_4220y
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:’’’’’’’’’: : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4235:$ 

_user_specified_name4233:W S
(
_output_shapes
:’’’’’’’’’
'
_user_specified_namedense_1_input

”
)__inference_sequential_layer_call_fn_4162
flatten_input
unknown:
	unknown_0:	
identity¢StatefulPartitionedCallį
StatefulPartitionedCallStatefulPartitionedCallflatten_inputunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *M
fHRF
D__inference_sequential_layer_call_and_return_conditional_losses_4143p
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*(
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*4
_input_shapes#
!:’’’’’’’’’: : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4158:$ 

_user_specified_name4156:` \
1
_output_shapes
:’’’’’’’’’
'
_user_specified_nameflatten_input
Ū

÷
A__inference_dense_1_layer_call_and_return_conditional_losses_4383

inputs3
matmul_readvariableop_resource:/
biasadd_readvariableop_resource:

identity¢BiasAdd/ReadVariableOp¢MatMul/ReadVariableOpw
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*!
_output_shapes
:*
dtype0k
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*)
_output_shapes
:’’’’’’’’’t
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes

:*
dtype0x
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*)
_output_shapes
:’’’’’’’’’X
SigmoidSigmoidBiasAdd:output:0*
T0*)
_output_shapes
:’’’’’’’’’\
IdentityIdentitySigmoid:y:0^NoOp*
T0*)
_output_shapes
:’’’’’’’’’S
NoOpNoOp^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*+
_input_shapes
:’’’’’’’’’: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:($
"
_user_specified_name
resource:($
"
_user_specified_name
resource:P L
(
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
®
Ę
D__inference_sequential_layer_call_and_return_conditional_losses_4153
flatten_input

dense_4147:

dense_4149:	
identity¢dense/StatefulPartitionedCall½
flatten/PartitionedCallPartitionedCallflatten_input*
Tin
2*
Tout
2*
_collective_manager_ids
 *)
_output_shapes
:’’’’’’’’’* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_flatten_layer_call_and_return_conditional_losses_4124ł
dense/StatefulPartitionedCallStatefulPartitionedCall flatten/PartitionedCall:output:0
dense_4147
dense_4149*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *H
fCRA
?__inference_dense_layer_call_and_return_conditional_losses_4136v
IdentityIdentity&dense/StatefulPartitionedCall:output:0^NoOp*
T0*(
_output_shapes
:’’’’’’’’’B
NoOpNoOp^dense/StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*4
_input_shapes#
!:’’’’’’’’’: : 2>
dense/StatefulPartitionedCalldense/StatefulPartitionedCall:$ 

_user_specified_name4149:$ 

_user_specified_name4147:` \
1
_output_shapes
:’’’’’’’’’
'
_user_specified_nameflatten_input
š
Ā
E__inference_autoencoder_layer_call_and_return_conditional_losses_4276
input_1$
sequential_4265:
sequential_4267:	&
sequential_1_4270:!
sequential_1_4272:

identity¢"sequential/StatefulPartitionedCall¢$sequential_1/StatefulPartitionedCallō
"sequential/StatefulPartitionedCallStatefulPartitionedCallinput_1sequential_4265sequential_4267*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *M
fHRF
D__inference_sequential_layer_call_and_return_conditional_losses_4143©
$sequential_1/StatefulPartitionedCallStatefulPartitionedCall+sequential/StatefulPartitionedCall:output:0sequential_1_4270sequential_1_4272*
Tin
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *O
fJRH
F__inference_sequential_1_layer_call_and_return_conditional_losses_4220
IdentityIdentity-sequential_1/StatefulPartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’n
NoOpNoOp#^sequential/StatefulPartitionedCall%^sequential_1/StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*8
_input_shapes'
%:’’’’’’’’’: : : : 2H
"sequential/StatefulPartitionedCall"sequential/StatefulPartitionedCall2L
$sequential_1/StatefulPartitionedCall$sequential_1/StatefulPartitionedCall:$ 

_user_specified_name4272:$ 

_user_specified_name4270:$ 

_user_specified_name4267:$ 

_user_specified_name4265:Z V
1
_output_shapes
:’’’’’’’’’
!
_user_specified_name	input_1
Ą	
×
*__inference_autoencoder_layer_call_fn_4303
input_1
unknown:
	unknown_0:	
	unknown_1:
	unknown_2:

identity¢StatefulPartitionedCall’
StatefulPartitionedCallStatefulPartitionedCallinput_1unknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *1
_output_shapes
:’’’’’’’’’*&
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *N
fIRG
E__inference_autoencoder_layer_call_and_return_conditional_losses_4276y
IdentityIdentity StatefulPartitionedCall:output:0^NoOp*
T0*1
_output_shapes
:’’’’’’’’’<
NoOpNoOp^StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*8
_input_shapes'
%:’’’’’’’’’: : : : 22
StatefulPartitionedCallStatefulPartitionedCall:$ 

_user_specified_name4299:$ 

_user_specified_name4297:$ 

_user_specified_name4295:$ 

_user_specified_name4293:Z V
1
_output_shapes
:’’’’’’’’’
!
_user_specified_name	input_1
L
Ś	
 __inference__traced_restore_4577
file_prefix2
assignvariableop_dense_kernel:,
assignvariableop_1_dense_bias:	6
!assignvariableop_2_dense_1_kernel:/
assignvariableop_3_dense_1_bias:
&
assignvariableop_4_iteration:	 *
 assignvariableop_5_learning_rate: ;
&assignvariableop_6_adam_m_dense_kernel:;
&assignvariableop_7_adam_v_dense_kernel:3
$assignvariableop_8_adam_m_dense_bias:	3
$assignvariableop_9_adam_v_dense_bias:	>
)assignvariableop_10_adam_m_dense_1_kernel:>
)assignvariableop_11_adam_v_dense_1_kernel:7
'assignvariableop_12_adam_m_dense_1_bias:
7
'assignvariableop_13_adam_v_dense_1_bias:
#
assignvariableop_14_total: #
assignvariableop_15_count: 
identity_17¢AssignVariableOp¢AssignVariableOp_1¢AssignVariableOp_10¢AssignVariableOp_11¢AssignVariableOp_12¢AssignVariableOp_13¢AssignVariableOp_14¢AssignVariableOp_15¢AssignVariableOp_2¢AssignVariableOp_3¢AssignVariableOp_4¢AssignVariableOp_5¢AssignVariableOp_6¢AssignVariableOp_7¢AssignVariableOp_8¢AssignVariableOp_9
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*¾
value“B±B&variables/0/.ATTRIBUTES/VARIABLE_VALUEB&variables/1/.ATTRIBUTES/VARIABLE_VALUEB&variables/2/.ATTRIBUTES/VARIABLE_VALUEB&variables/3/.ATTRIBUTES/VARIABLE_VALUEB0optimizer/_iterations/.ATTRIBUTES/VARIABLE_VALUEB3optimizer/_learning_rate/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/1/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/2/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/3/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/4/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/5/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/6/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/7/.ATTRIBUTES/VARIABLE_VALUEB1optimizer/_variables/8/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*5
value,B*B B B B B B B B B B B B B B B B B ó
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*X
_output_shapesF
D:::::::::::::::::*
dtypes
2	[
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0*
_output_shapes
:°
AssignVariableOpAssignVariableOpassignvariableop_dense_kernelIdentity:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0]

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:“
AssignVariableOp_1AssignVariableOpassignvariableop_1_dense_biasIdentity_1:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0]

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:ø
AssignVariableOp_2AssignVariableOp!assignvariableop_2_dense_1_kernelIdentity_2:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0]

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:¶
AssignVariableOp_3AssignVariableOpassignvariableop_3_dense_1_biasIdentity_3:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0]

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0	*
_output_shapes
:³
AssignVariableOp_4AssignVariableOpassignvariableop_4_iterationIdentity_4:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0	]

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:·
AssignVariableOp_5AssignVariableOp assignvariableop_5_learning_rateIdentity_5:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0]

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:½
AssignVariableOp_6AssignVariableOp&assignvariableop_6_adam_m_dense_kernelIdentity_6:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0]

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:½
AssignVariableOp_7AssignVariableOp&assignvariableop_7_adam_v_dense_kernelIdentity_7:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0]

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0*
_output_shapes
:»
AssignVariableOp_8AssignVariableOp$assignvariableop_8_adam_m_dense_biasIdentity_8:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0]

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:»
AssignVariableOp_9AssignVariableOp$assignvariableop_9_adam_v_dense_biasIdentity_9:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0_
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:Ā
AssignVariableOp_10AssignVariableOp)assignvariableop_10_adam_m_dense_1_kernelIdentity_10:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0_
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:Ā
AssignVariableOp_11AssignVariableOp)assignvariableop_11_adam_v_dense_1_kernelIdentity_11:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0_
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0*
_output_shapes
:Ą
AssignVariableOp_12AssignVariableOp'assignvariableop_12_adam_m_dense_1_biasIdentity_12:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0_
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:Ą
AssignVariableOp_13AssignVariableOp'assignvariableop_13_adam_v_dense_1_biasIdentity_13:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0_
Identity_14IdentityRestoreV2:tensors:14"/device:CPU:0*
T0*
_output_shapes
:²
AssignVariableOp_14AssignVariableOpassignvariableop_14_totalIdentity_14:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0_
Identity_15IdentityRestoreV2:tensors:15"/device:CPU:0*
T0*
_output_shapes
:²
AssignVariableOp_15AssignVariableOpassignvariableop_15_countIdentity_15:output:0"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 *
dtype0Y
NoOpNoOp"/device:CPU:0*&
 _has_manual_control_dependencies(*
_output_shapes
 Æ
Identity_16Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9^NoOp"/device:CPU:0*
T0*
_output_shapes
: W
Identity_17IdentityIdentity_16:output:0^NoOp_1*
T0*
_output_shapes
: ų
NoOp_1NoOp^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9*
_output_shapes
 "#
identity_17Identity_17:output:0*(
_construction_contextkEagerRuntime*5
_input_shapes$
": : : : : : : : : : : : : : : : : 2*
AssignVariableOp_10AssignVariableOp_102*
AssignVariableOp_11AssignVariableOp_112*
AssignVariableOp_12AssignVariableOp_122*
AssignVariableOp_13AssignVariableOp_132*
AssignVariableOp_14AssignVariableOp_142*
AssignVariableOp_15AssignVariableOp_152(
AssignVariableOp_1AssignVariableOp_12(
AssignVariableOp_2AssignVariableOp_22(
AssignVariableOp_3AssignVariableOp_32(
AssignVariableOp_4AssignVariableOp_42(
AssignVariableOp_5AssignVariableOp_52(
AssignVariableOp_6AssignVariableOp_62(
AssignVariableOp_7AssignVariableOp_72(
AssignVariableOp_8AssignVariableOp_82(
AssignVariableOp_9AssignVariableOp_92$
AssignVariableOpAssignVariableOp:%!

_user_specified_namecount:%!

_user_specified_nametotal:3/
-
_user_specified_nameAdam/v/dense_1/bias:3/
-
_user_specified_nameAdam/m/dense_1/bias:51
/
_user_specified_nameAdam/v/dense_1/kernel:51
/
_user_specified_nameAdam/m/dense_1/kernel:1
-
+
_user_specified_nameAdam/v/dense/bias:1	-
+
_user_specified_nameAdam/m/dense/bias:3/
-
_user_specified_nameAdam/v/dense/kernel:3/
-
_user_specified_nameAdam/m/dense/kernel:-)
'
_user_specified_namelearning_rate:)%
#
_user_specified_name	iteration:,(
&
_user_specified_namedense_1/bias:.*
(
_user_specified_namedense_1/kernel:*&
$
_user_specified_name
dense/bias:,(
&
_user_specified_namedense/kernel:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
É
]
A__inference_flatten_layer_call_and_return_conditional_losses_4343

inputs
identityV
ConstConst*
_output_shapes
:*
dtype0*
valueB"’’’’ Ą  ^
ReshapeReshapeinputsConst:output:0*
T0*)
_output_shapes
:’’’’’’’’’Z
IdentityIdentityReshape:output:0*
T0*)
_output_shapes
:’’’’’’’’’"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*0
_input_shapes
:’’’’’’’’’:Y U
1
_output_shapes
:’’’’’’’’’
 
_user_specified_nameinputs
®
Ę
D__inference_sequential_layer_call_and_return_conditional_losses_4143
flatten_input

dense_4137:

dense_4139:	
identity¢dense/StatefulPartitionedCall½
flatten/PartitionedCallPartitionedCallflatten_input*
Tin
2*
Tout
2*
_collective_manager_ids
 *)
_output_shapes
:’’’’’’’’’* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8 *J
fERC
A__inference_flatten_layer_call_and_return_conditional_losses_4124ł
dense/StatefulPartitionedCallStatefulPartitionedCall flatten/PartitionedCall:output:0
dense_4137
dense_4139*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:’’’’’’’’’*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8 *H
fCRA
?__inference_dense_layer_call_and_return_conditional_losses_4136v
IdentityIdentity&dense/StatefulPartitionedCall:output:0^NoOp*
T0*(
_output_shapes
:’’’’’’’’’B
NoOpNoOp^dense/StatefulPartitionedCall*
_output_shapes
 "
identityIdentity:output:0*(
_construction_contextkEagerRuntime*4
_input_shapes#
!:’’’’’’’’’: : 2>
dense/StatefulPartitionedCalldense/StatefulPartitionedCall:$ 

_user_specified_name4139:$ 

_user_specified_name4137:` \
1
_output_shapes
:’’’’’’’’’
'
_user_specified_nameflatten_input"ķL
saver_filename:0StatefulPartitionedCall_1:0StatefulPartitionedCall_28"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*æ
serving_default«
E
input_1:
serving_default_input_1:0’’’’’’’’’F
output_1:
StatefulPartitionedCall:0’’’’’’’’’tensorflow/serving/predict:
ū
	variables
trainable_variables
regularization_losses
	keras_api
__call__
*&call_and_return_all_conditional_losses
_default_save_signature
encoder
	decoder

	optimizer

signatures"
_tf_keras_model
<
0
1
2
3"
trackable_list_wrapper
<
0
1
2
3"
trackable_list_wrapper
 "
trackable_list_wrapper
Ź
non_trainable_variables

layers
metrics
layer_regularization_losses
layer_metrics
	variables
trainable_variables
regularization_losses
__call__
_default_save_signature
*&call_and_return_all_conditional_losses
&"call_and_return_conditional_losses"
_generic_user_object
Ā
trace_0
trace_12
*__inference_autoencoder_layer_call_fn_4303
*__inference_autoencoder_layer_call_fn_4316°
©²„
FullArgSpec
args
jx
varargs
 
varkw
 
defaults
 

kwonlyargs

jtraining%
kwonlydefaultsŖ

trainingp 
annotationsŖ *
 ztrace_0ztrace_1
ų
trace_0
trace_12Į
E__inference_autoencoder_layer_call_and_return_conditional_losses_4276
E__inference_autoencoder_layer_call_and_return_conditional_losses_4290°
©²„
FullArgSpec
args
jx
varargs
 
varkw
 
defaults
 

kwonlyargs

jtraining%
kwonlydefaultsŖ

trainingp 
annotationsŖ *
 ztrace_0ztrace_1
ŹBĒ
__inference__wrapped_model_4116input_1"
²
FullArgSpec
args

jargs_0
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
Ž
layer-0
layer_with_weights-0
layer-1
	variables
trainable_variables
regularization_losses
	keras_api
__call__
* &call_and_return_all_conditional_losses"
_tf_keras_sequential
Ž
!layer_with_weights-0
!layer-0
"layer-1
#	variables
$trainable_variables
%regularization_losses
&	keras_api
'__call__
*(&call_and_return_all_conditional_losses"
_tf_keras_sequential

)
_variables
*_iterations
+_learning_rate
,_index_dict
-
_momentums
._velocities
/_update_step_xla"
experimentalOptimizer
,
0serving_default"
signature_map
!:2dense/kernel
:2
dense/bias
#:!2dense_1/kernel
:2dense_1/bias
 "
trackable_list_wrapper
.
0
	1"
trackable_list_wrapper
'
10"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
ÜBŁ
*__inference_autoencoder_layer_call_fn_4303input_1"
²
FullArgSpec
args
jx
varargs
 
varkw
 
defaults
 

kwonlyargs

jtraining
kwonlydefaults
 
annotationsŖ *
 
ÜBŁ
*__inference_autoencoder_layer_call_fn_4316input_1"
²
FullArgSpec
args
jx
varargs
 
varkw
 
defaults
 

kwonlyargs

jtraining
kwonlydefaults
 
annotationsŖ *
 
÷Bō
E__inference_autoencoder_layer_call_and_return_conditional_losses_4276input_1"
²
FullArgSpec
args
jx
varargs
 
varkw
 
defaults
 

kwonlyargs

jtraining
kwonlydefaults
 
annotationsŖ *
 
÷Bō
E__inference_autoencoder_layer_call_and_return_conditional_losses_4290input_1"
²
FullArgSpec
args
jx
varargs
 
varkw
 
defaults
 

kwonlyargs

jtraining
kwonlydefaults
 
annotationsŖ *
 
„
2	variables
3trainable_variables
4regularization_losses
5	keras_api
6__call__
*7&call_and_return_all_conditional_losses"
_tf_keras_layer
»
8	variables
9trainable_variables
:regularization_losses
;	keras_api
<__call__
*=&call_and_return_all_conditional_losses

kernel
bias"
_tf_keras_layer
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
­
>non_trainable_variables

?layers
@metrics
Alayer_regularization_losses
Blayer_metrics
	variables
trainable_variables
regularization_losses
__call__
* &call_and_return_all_conditional_losses
& "call_and_return_conditional_losses"
_generic_user_object
Å
Ctrace_0
Dtrace_12
)__inference_sequential_layer_call_fn_4162
)__inference_sequential_layer_call_fn_4171µ
®²Ŗ
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults¢
p 

 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 zCtrace_0zDtrace_1
ū
Etrace_0
Ftrace_12Ä
D__inference_sequential_layer_call_and_return_conditional_losses_4143
D__inference_sequential_layer_call_and_return_conditional_losses_4153µ
®²Ŗ
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults¢
p 

 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 zEtrace_0zFtrace_1
»
G	variables
Htrainable_variables
Iregularization_losses
J	keras_api
K__call__
*L&call_and_return_all_conditional_losses

kernel
bias"
_tf_keras_layer
„
M	variables
Ntrainable_variables
Oregularization_losses
P	keras_api
Q__call__
*R&call_and_return_all_conditional_losses"
_tf_keras_layer
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
­
Snon_trainable_variables

Tlayers
Umetrics
Vlayer_regularization_losses
Wlayer_metrics
#	variables
$trainable_variables
%regularization_losses
'__call__
*(&call_and_return_all_conditional_losses
&("call_and_return_conditional_losses"
_generic_user_object
É
Xtrace_0
Ytrace_12
+__inference_sequential_1_layer_call_fn_4239
+__inference_sequential_1_layer_call_fn_4248µ
®²Ŗ
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults¢
p 

 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 zXtrace_0zYtrace_1
’
Ztrace_0
[trace_12Č
F__inference_sequential_1_layer_call_and_return_conditional_losses_4220
F__inference_sequential_1_layer_call_and_return_conditional_losses_4230µ
®²Ŗ
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults¢
p 

 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 zZtrace_0z[trace_1
_
*0
\1
]2
^3
_4
`5
a6
b7
c8"
trackable_list_wrapper
:	 2	iteration
: 2learning_rate
 "
trackable_dict_wrapper
<
\0
^1
`2
b3"
trackable_list_wrapper
<
]0
_1
a2
c3"
trackable_list_wrapper
µ2²Æ
¦²¢
FullArgSpec*
args"

jgradient

jvariable
jkey
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 0
ĪBĖ
"__inference_signature_wrapper_4332input_1"
²
FullArgSpec
args 
varargs
 
varkw
 
defaults
 

kwonlyargs
	jinput_1
kwonlydefaults
 
annotationsŖ *
 
N
d	variables
e	keras_api
	ftotal
	gcount"
_tf_keras_metric
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
­
hnon_trainable_variables

ilayers
jmetrics
klayer_regularization_losses
llayer_metrics
2	variables
3trainable_variables
4regularization_losses
6__call__
*7&call_and_return_all_conditional_losses
&7"call_and_return_conditional_losses"
_generic_user_object
ą
mtrace_02Ć
&__inference_flatten_layer_call_fn_4337
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 zmtrace_0
ū
ntrace_02Ž
A__inference_flatten_layer_call_and_return_conditional_losses_4343
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 zntrace_0
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
­
onon_trainable_variables

players
qmetrics
rlayer_regularization_losses
slayer_metrics
8	variables
9trainable_variables
:regularization_losses
<__call__
*=&call_and_return_all_conditional_losses
&="call_and_return_conditional_losses"
_generic_user_object
Ž
ttrace_02Į
$__inference_dense_layer_call_fn_4352
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 zttrace_0
ł
utrace_02Ü
?__inference_dense_layer_call_and_return_conditional_losses_4363
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 zutrace_0
 "
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
īBė
)__inference_sequential_layer_call_fn_4162flatten_input"¬
„²”
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
īBė
)__inference_sequential_layer_call_fn_4171flatten_input"¬
„²”
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
B
D__inference_sequential_layer_call_and_return_conditional_losses_4143flatten_input"¬
„²”
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
B
D__inference_sequential_layer_call_and_return_conditional_losses_4153flatten_input"¬
„²”
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
­
vnon_trainable_variables

wlayers
xmetrics
ylayer_regularization_losses
zlayer_metrics
G	variables
Htrainable_variables
Iregularization_losses
K__call__
*L&call_and_return_all_conditional_losses
&L"call_and_return_conditional_losses"
_generic_user_object
ą
{trace_02Ć
&__inference_dense_1_layer_call_fn_4372
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 z{trace_0
ū
|trace_02Ž
A__inference_dense_1_layer_call_and_return_conditional_losses_4383
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 z|trace_0
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
Æ
}non_trainable_variables

~layers
metrics
 layer_regularization_losses
layer_metrics
M	variables
Ntrainable_variables
Oregularization_losses
Q__call__
*R&call_and_return_all_conditional_losses
&R"call_and_return_conditional_losses"
_generic_user_object
ā
trace_02Ć
&__inference_reshape_layer_call_fn_4388
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 ztrace_0
ż
trace_02Ž
A__inference_reshape_layer_call_and_return_conditional_losses_4402
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 ztrace_0
 "
trackable_list_wrapper
.
!0
"1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
šBķ
+__inference_sequential_1_layer_call_fn_4239dense_1_input"¬
„²”
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
šBķ
+__inference_sequential_1_layer_call_fn_4248dense_1_input"¬
„²”
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
B
F__inference_sequential_1_layer_call_and_return_conditional_losses_4220dense_1_input"¬
„²”
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
B
F__inference_sequential_1_layer_call_and_return_conditional_losses_4230dense_1_input"¬
„²”
FullArgSpec)
args!
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
&:$2Adam/m/dense/kernel
&:$2Adam/v/dense/kernel
:2Adam/m/dense/bias
:2Adam/v/dense/bias
(:&2Adam/m/dense_1/kernel
(:&2Adam/v/dense_1/kernel
!:2Adam/m/dense_1/bias
!:2Adam/v/dense_1/bias
.
f0
g1"
trackable_list_wrapper
-
d	variables"
_generic_user_object
:  (2total
:  (2count
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
ŠBĶ
&__inference_flatten_layer_call_fn_4337inputs"
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
ėBč
A__inference_flatten_layer_call_and_return_conditional_losses_4343inputs"
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
ĪBĖ
$__inference_dense_layer_call_fn_4352inputs"
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
éBę
?__inference_dense_layer_call_and_return_conditional_losses_4363inputs"
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
ŠBĶ
&__inference_dense_1_layer_call_fn_4372inputs"
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
ėBč
A__inference_dense_1_layer_call_and_return_conditional_losses_4383inputs"
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
ŠBĶ
&__inference_reshape_layer_call_fn_4388inputs"
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 
ėBč
A__inference_reshape_layer_call_and_return_conditional_losses_4402inputs"
²
FullArgSpec
args

jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsŖ *
 „
__inference__wrapped_model_4116:¢7
0¢-
+(
input_1’’’’’’’’’
Ŗ "=Ŗ:
8
output_1,)
output_1’’’’’’’’’Ō
E__inference_autoencoder_layer_call_and_return_conditional_losses_4276J¢G
0¢-
+(
input_1’’’’’’’’’
Ŗ

trainingp"6¢3
,)
tensor_0’’’’’’’’’
 Ō
E__inference_autoencoder_layer_call_and_return_conditional_losses_4290J¢G
0¢-
+(
input_1’’’’’’’’’
Ŗ

trainingp "6¢3
,)
tensor_0’’’’’’’’’
 ­
*__inference_autoencoder_layer_call_fn_4303J¢G
0¢-
+(
input_1’’’’’’’’’
Ŗ

trainingp"+(
unknown’’’’’’’’’­
*__inference_autoencoder_layer_call_fn_4316J¢G
0¢-
+(
input_1’’’’’’’’’
Ŗ

trainingp "+(
unknown’’’’’’’’’«
A__inference_dense_1_layer_call_and_return_conditional_losses_4383f0¢-
&¢#
!
inputs’’’’’’’’’
Ŗ ".¢+
$!
tensor_0’’’’’’’’’
 
&__inference_dense_1_layer_call_fn_4372[0¢-
&¢#
!
inputs’’’’’’’’’
Ŗ "# 
unknown’’’’’’’’’©
?__inference_dense_layer_call_and_return_conditional_losses_4363f1¢.
'¢$
"
inputs’’’’’’’’’
Ŗ "-¢*
# 
tensor_0’’’’’’’’’
 
$__inference_dense_layer_call_fn_4352[1¢.
'¢$
"
inputs’’’’’’’’’
Ŗ ""
unknown’’’’’’’’’°
A__inference_flatten_layer_call_and_return_conditional_losses_4343k9¢6
/¢,
*'
inputs’’’’’’’’’
Ŗ ".¢+
$!
tensor_0’’’’’’’’’
 
&__inference_flatten_layer_call_fn_4337`9¢6
/¢,
*'
inputs’’’’’’’’’
Ŗ "# 
unknown’’’’’’’’’°
A__inference_reshape_layer_call_and_return_conditional_losses_4402k1¢.
'¢$
"
inputs’’’’’’’’’
Ŗ "6¢3
,)
tensor_0’’’’’’’’’
 
&__inference_reshape_layer_call_fn_4388`1¢.
'¢$
"
inputs’’’’’’’’’
Ŗ "+(
unknown’’’’’’’’’Ē
F__inference_sequential_1_layer_call_and_return_conditional_losses_4220}?¢<
5¢2
(%
dense_1_input’’’’’’’’’
p

 
Ŗ "6¢3
,)
tensor_0’’’’’’’’’
 Ē
F__inference_sequential_1_layer_call_and_return_conditional_losses_4230}?¢<
5¢2
(%
dense_1_input’’’’’’’’’
p 

 
Ŗ "6¢3
,)
tensor_0’’’’’’’’’
 ”
+__inference_sequential_1_layer_call_fn_4239r?¢<
5¢2
(%
dense_1_input’’’’’’’’’
p

 
Ŗ "+(
unknown’’’’’’’’’”
+__inference_sequential_1_layer_call_fn_4248r?¢<
5¢2
(%
dense_1_input’’’’’’’’’
p 

 
Ŗ "+(
unknown’’’’’’’’’Å
D__inference_sequential_layer_call_and_return_conditional_losses_4143}H¢E
>¢;
1.
flatten_input’’’’’’’’’
p

 
Ŗ "-¢*
# 
tensor_0’’’’’’’’’
 Å
D__inference_sequential_layer_call_and_return_conditional_losses_4153}H¢E
>¢;
1.
flatten_input’’’’’’’’’
p 

 
Ŗ "-¢*
# 
tensor_0’’’’’’’’’
 
)__inference_sequential_layer_call_fn_4162rH¢E
>¢;
1.
flatten_input’’’’’’’’’
p

 
Ŗ ""
unknown’’’’’’’’’
)__inference_sequential_layer_call_fn_4171rH¢E
>¢;
1.
flatten_input’’’’’’’’’
p 

 
Ŗ ""
unknown’’’’’’’’’³
"__inference_signature_wrapper_4332E¢B
¢ 
;Ŗ8
6
input_1+(
input_1’’’’’’’’’"=Ŗ:
8
output_1,)
output_1’’’’’’’’’