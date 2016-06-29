FUNCTION b0{
	PARAMETER j.
	PARAMETER t_j.
	LOCAL OUT IS -v_e[j]*ln(1-t_j/tau_[j]).
	RETURN OUT.
}
FUNCTION b_{
	PARAMETER n.
	PARAMETER j.
	PARAMETER t_j.
	IF n = 0{ RETURN b0(j,t_j).}
	LOCAL OUT IS b_(n-1,j,t_j)*tau_[j] - v_e[j]*t_j^n/n.
	RETURN OUT.
}
FUNCTION c0{
	PARAMETER j.
	PARAMETER t_j.
	LOCAL OUT IS b0(j,t_j)*t_j - b_(1,j,t_j).
	RETURN OUT.
}
FUNCTION c_{
	PARAMETER n.
	PARAMETER j.
	PARAMETER t_j.
	IF n = 0 {RETURN c0(j,t_j).}
	LOCAL OUT IS c_(n-1,j,t_j)*tau_[j] - v_e[j]*t_j^(n+1)/(n*(n+1)).
	RETURN OUT.
}
FUNCTION a0{
	PARAMETER j.
	PARAMETER t_j.
	LOCAL OUT IS acc_[j]/(1-t_j/tau_[j]).
	RETURN OUT.
}