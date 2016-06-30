GLOBAL t0 IS 0.
GLOBAL t IS -10.
GLOBAL g0 IS BODY:MU/(BODY:RADIUS+SHIP:ALTITUDE)^2.
GLOBAL mu IS BODY:MU.
GLOBAL Re IS BODY:RADIUS.
GLOBAL rvec IS v(SHIP:ALTITUDE+BODY:RADIUS,0,0).
GLOBAL vvec IS v(SHIP:VERTICALSPEED,SHIP:VELOCITY:OBT:MAG,0).
GLOBAL fhat IS v(0,0,0):normalized.
GLOBAL r_T IS LIST(0,0,0).
GLOBAL rdot_T IS LIST(0,0,0).
GLOBAL v_thetaT IS LIST(0,0,0).
GLOBAL omega_T IS LIST(0,0,0).
GLOBAL h_T IS LIST(0,0,0).
GLOBAL energy_T IS LIST(0,0,0).
GLOBAL A_in IS LIST(0,0,0).
GLOBAL B_in IS LIST(0,0,0).
GLOBAL T_in IS LIST(0,0,0).
GLOBAL A_upd IS LIST(0,0,0).
GLOBAL B_upd IS LIST(0,0,0).
GLOBAL T_upd IS LIST(0,0,0).
GLOBAL r_ IS 0.
GLOBAL v_ IS 0.
GLOBAL z_ IS 0.
GLOBAL hvec IS v(0,0,0).
GLOBAL h_ IS 0.
GLOBAL rhat IS v(0,0,0):normalized.
GLOBAL hhat IS v(0,0,0):normalized.
GLOBAL thetahat IS v(0,0,0):normalized.
GLOBAL thrust_ IS 0.
GLOBAL mass_ IS 0.
GLOBAL acc_ IS LIST(0,0,0.8*g0).
GLOBAL v_e IS LIST(0,0,348*g0).
GLOBAL tau_ IS LIST(0,0,v_e[2]/acc_[2]).
GLOBAL omega IS 0.
GLOBAL rdot IS 0.
GLOBAL vtheta IS 0.
GLOBAL Delta_t IS 0.
GLOBAL tlast IS 0.
GLOBAL Delta_h IS 0.
GLOBAL rbar_ IS LIST(0,0,0).
GLOBAL f_r IS LIST(0,0,0).
GLOBAL f_rT IS LIST(0,0,0).
GLOBAL a_T IS LIST(0,0,0).
GLOBAL fdot_r IS LIST(0,0,0).
GLOBAL f_theta IS LIST(0,0,0).
GLOBAL fdot_theta IS LIST(0,0,0).
GLOBAL fdotdot_theta IS LIST(0,0,0).
GLOBAL N1 IS 0.
GLOBAL N2a IS 0.
GLOBAL N2b IS 0.
GLOBAL N2 IS 0.
GLOBAL N3 IS 0.
GLOBAL N IS 0.
GLOBAL D0 IS 0.
GLOBAL D1 IS 0.
GLOBAL D2 IS 0.
GLOBAL D IS 0.
GLOBAL Delta_v IS 0.
GLOBAL k_b IS 0.
GLOBAL k_c IS 0.
GLOBAL aa IS 0.
GLOBAL bb IS 0.
GLOBAL cc IS 0.
GLOBAL dd IS 0.
GLOBAL Delta_A IS LIST(0,0,0).
GLOBAL Delta_B IS LIST(0,0,0).
GLOBAL A_ IS LIST(0,0,0).
GLOBAL B_ IS LIST(0,0,0).
GLOBAL C_ IS LIST(0,0,0).
GLOBAL C_T IS LIST(0,0,0).
GLOBAL T_ IS LIST(0,305,400).
GLOBAL g IS LIST(0,0,0).
GLOBAL cent IS 0.
GLOBAL g_term IS 0.
GLOBAL cent_term IS 0.
GLOBAL fhatdot_rhat IS 0.
GLOBAL fhatdot_hhat IS 0.
GLOBAL fhatdot_thetahat IS 0.
GLOBAL energy IS 0.
GLOBAL Delta_energy IS LIST(0,0,0).
GLOBAL energyTime IS LIST(0,0,0).
GLOBAL t_cutoff IS 0.
GLOBAL d_3 IS 0.
GLOBAL d_3T IS 0.
GLOBAL d_4 IS 0.
GLOBAL Delta_theta IS 0.
GLOBAL sintheta IS 0.
GLOBAL costheta IS 0.
GLOBAL theta IS 0.
GLOBAL eta_T IS 0.
GLOBAL v_T IS LIST(0,0,0).
GLOBAL j IS 0.
GLOBAL i IS 0.
GLOBAL cutoffEnable IS FALSE.
LOCAL Nstage IS 0.

FUNCTION peg_Init{
	PARAMETER StageNumber.
	PARAMETER Lr_T.
	PARAMETER Lrdot_T.
	PARAMETER Lvtheta_T.
	PARAMETER Lt.
	SET Nstage TO StageNumber.
	SET r_T[nStage] TO Lr_T.
	SET rdot_T[nStage] TO Lrdot_T.
	SET v_thetaT[nStage] TO Lvtheta_T.
	SET h_T[nStage] TO vcrs(v(r_T[nStage],0,0),v(rdot_T[nStage],v_thetaT[nStage],0)):MAG.
	SET omega_T[nStage] TO v_thetaT[nStage]/r_T[nStage].
	SET energy_T[nStage] TO v_thetaT[nStage]^2 + rdot_T[nStage]^2 - 2*mu/r_T[nStage].
	SET tlast TO Lt.
	SET j to 1.
}
FUNCTION MajorCycle{
	Navigate().
	IF acc_[j] > 1 {
		Estimate().
		GuideMajor().
	}
}
FUNCTION Estimate{
	EstimateStaging().
	SET cutoffEnable TO T_[nStage]<10.
	EstimateT().
	EstimateDelta_theta().
}
FUNCTION MinorCycle{
	GuideMinor().
}

FUNCTION Navigate{
	LOCAL e_thr IS 0.
	LOCAL e_isp IS 0.
	FOR e IN engines {
		//	sum specific impulses of all active engines:
		//	Isp_sum = sum(Isp_i*thrust_i) / sum(thrust_i)
		//	note that the following does not do division, only two sums
		IF e:IGNITION {
			SET e_thr TO e_thr + e:THRUST.
			SET e_isp TO e:ISP.
		}.
	}.
	SET rvec TO v(SHIP:ALTITUDE+BODY:RADIUS,0,0).
	SET vvec TO v(SHIP:VERTICALSPEED,SHIP:VELOCITY:OBT:MAG,0).
	SET r_ TO SHIP:ALTITUDE+BODY:RADIUS.
	SET v_ TO vvec:mag.
	SET z_ TO r_-Re.
	SET hvec TO vcrs(rvec,vvec).
	SET h_ TO hvec:mag.
	SET rhat TO rvec:normalized.
	SET hhat TO hvec:normalized.
	SET thetahat TO SHIP:VELOCITY:OBT:normalized.
	SET thrust_ TO e_thr.
	SET mass_ TO SHIP:MASS.
	SET acc_[j] TO thrust_/mass_.
	SET v_e[j] TO e_isp*g0.
	IF acc_[j] > 1 {SET tau_[j] TO v_e[j]/acc_[j].}
	SET omega TO h_/(r_*r_).
	IF omega_T[j] = 0 {SET omega_T[j] TO omega.}
	SET rdot TO SHIP:VERTICALSPEED.
	SET vtheta TO SHIP:VELOCITY:ORBIT:MAG.
	SET Delta_t TO t-tlast.
	SET tlast TO t.
	FROM {SET i TO 1.} UNTIL i = nStage STEP { SET i TO i+1.} DO{
		SET A_in[i] TO A_[i].
		SET B_in[i] TO B_[i].
		SET T_in[i] TO T_[i].
	}
	SET f_r[j] TO f_r[j] + fdot_r[j]*Delta_t.
	SET A_[j] TO A_[j] + B_[j] * Delta_t.
	SET T_[j] TO T_[j] - Delta_t.
	FROM {SET i TO 1.} UNTIL i = nStage STEP {SET i TO i+1.} DO{
		SET A_upd[i] TO A_[i].
		SET B_upd[i] TO B_[i].
		SET T_upd[i] TO T_[i].
	}
	IF T_[j] < 1 {
		SET T_[j] TO 1.
		IF j < nStage AND acc_[j] < 1{
			SET A_[j] TO A_[j] + Delta_A[j].
			SET B_[j] TO B_[j] + Delta_B[j].
			//IF acc_[j] < 1{
			SET j TO j+1.
			STAGIN().
			//}
		}
	}
}


FUNCTION EstimateStaging{
	SET h_T[0] TO h_.
	SET omega_T[0] TO omega.
	SET r_T[0] TO r_.
	SET rdot_T[0] TO rdot.
	SET v_thetaT[0] TO vtheta.
	SET T_[0] TO 0.
	SET Delta_A[0] TO 0.
	SET Delta_B[0] TO 0.
	IF acc_[j] <= 1{
		STAGIN().
		return.
	}
	IF j < nStage {
		SET rdot_T[j] TO rdot+b0(j,T_[j])*A_[j]+b_(1,j,T_[j])*B_[j].
		SET r_T[j] TO r_+rdot*T_[j]+c_(0,j,T_[j])*A_[j]+c_(1,j,T_[j])*B_[j].
		SET rbar_[j] TO (r_T[j]+r_)/2.
		SET C_[j] TO (mu/r_^2 - vtheta^2/r_)/acc_[j].
		SET f_r[j] TO A_[j] + C_[j].
		SET a_T[j] TO a0(j,T_[j]).
		//SET C_T[j] TO (mu/r_^2 - omega_T[j]^2*r_)/a_T[j].
		//SET f_rT[j] TO A_[j]+B_[j]*T_[j]+ C_T[j].
		//SET fdot_r[j] TO (f_rT[j]-f_r[j])/T_[j].
		SET f_theta[j] TO 1-f_r[j]*f_r[j]/2.
		SET fdot_theta[j] TO -f_r[j]*fdot_r[j].
		SET fdotdot_theta[j] TO -fdot_r[j]*fdot_r[j]/2.
		SET h_T[j] TO h_+rbar_[j]*(f_theta[j]*b0(j,T_[j])+fdot_theta[j]*b_(1,j,T_[j])+fdotdot_theta[j]*b_(2,j,T_[j])).
		SET v_thetaT[j] TO h_T[j]/r_T[j].
		SET omega_T[j] TO v_thetaT[j]/r_T[j].
		SET Delta_A[j] TO (mu/r_^2-omega_T[j]^2*r_)*(1/a0(j,T_[j])-1/acc_[j+1]).
		SET Delta_B[j] TO (mu/r_^2-omega_T[j]^2*r_)*(1/v_e[j+1]-1/v_e[j])+(3*h_T[j]^2/r_T[j] - 2*mu)*rdot_T[j]/r_T[j]^3 * (1/a_T[j] - 1/acc_[j+1]).
		SET A_[j+1] TO A_[j] + Delta_A[j] + B_[j]*T_[j].
		SET B_[j+1] TO B_[j] + Delta_B[j].
	}
	ELSE {
		SET h_T[j-1] TO h_.
		SET omega_T[j-1] TO omega.
		SET r_T[j-1] TO r_.
		SET rdot_T[j-1] TO rdot.
		SET v_thetaT[j-1] TO vtheta.
		SET T_[j-1] TO 0.
		SET Delta_A[j-1] TO 0.
		SET Delta_B[j-1] TO 0.
	}	
}
FUNCTION EstimateT{
	IF acc_[j] <= 1{
		return.
	}
	//SET rdot_T[nStage] TO rdot + (b0(nStage-1,T_[nStage-1]) + b0(nStage,T_[nStage]))*A_[nStage-1] + (b_(1,nStage-1,T_[nStage-1]) + b_(1,nStage,T_[nStage]) + b0(nStage,T_[nStage])*T_[nStage-1])*B_[nStage-1] + b0(nStage,T_[nStage])*Delta_A[nStage-1] + b_(1,nStage,T_[nStage])*Delta_B[nStage-1].
	//SET r_T[nStage] TO r_ + rdot*T_[nStage] + (c0(nStage-1,T_[nStage-1]) + c0(nStage,T_[nStage]) + b0(nStage-1,T_[nStage-1])*T_[nStage])*A_[nStage-1] + (c_(1,nStage-1,T_[nStage-1]) + b_(1,nStage-1,T_[nStage-1])*T_[nStage] + c0(nStage,T_[nStage])*T_[nStage-1] + c_(1,nStage,T_[nStage]))*B_[nStage-1] + c0(nStage,T_[nStage])*Delta_A[nStage-1] + c_(1,nStage,T_[nStage])*Delta_B.
	SET rbar_[nStage] TO (r_T[nStage]+r_T[nStage-1])/2.
	SET Delta_h TO h_T[nStage] - h_T[nStage-1].
	SET C_[nStage] TO (mu/r_^2 - omega_T[nStage-1]^2/r_)/acc_[nStage].
	SET f_r[nStage] TO A_[nStage] + C_[nStage].
	SET a_T[nStage] TO a0(nStage,T_[nStage]).
	SET C_T[nStage] TO (mu/r_T[nStage]^2 - omega_T[nStage]^2/r_T[nStage])/a_T[nStage].
	SET f_rT[j] TO A_[j] + B_[j]*(T_[nStage] + T_[nStage-1]) + Delta_A[j] + Delta_B[j]*T_[nStage] + C_T[nStage].
	SET fdot_r[j] TO (f_rT[j] - f_r[j])/(T_[nStage]+T_[nStage-1]).
	SET f_theta[nStage] TO 1 - (f_r[nStage]^2)/2.
	SET fdot_theta[nStage] TO -(f_r[nStage]*fdot_r[j]).
	SET fdotdot_theta[nStage] TO -(fdot_r[j]^2)/2.
	SET N TO 2*Delta_h/(r_T[nStage]+r_T[nStage-1]) + v_e[nStage]*T_[nStage]*(fdot_theta[nStage]+fdotdot_theta[nStage]*(tau_[nStage]+T_[nStage]/2)).
	SET D TO f_theta[nStage] + fdot_theta[nStage]*tau_[nStage] + fdotdot_theta[nStage]*tau_[nStage]^2.
	IF ABS(D) < 1.0{
		SET Delta_v TO N/D.
	} ELSE {
		SET Delta_v TO Delta_h/rbar_[nStage].
	}
	IF NOT cutoffEnable{
		SET T_[nStage] TO tau_[nStage]*(1 - CONSTANT():E ^(-Delta_v/v_e[nStage])).
	}
	"IF T_[nStage] > 633{
		SET T_[nStage] TO 633.
	}".
}
FUNCTION EstimateDelta_theta{
	SET Delta_theta TO 0.
	FROM {SET i TO 1.} UNTIL i = 2 STEP{SET i TO i+1.} DO{
		IF T_[i] > 0{
			SET d_3 TO h_T[i-1]*rdot_T[i-1]/r_T[i-1]^3.
			SET d_3T TO h_T[i]*rdot_T[i]/r_T[i]^3.
			SET d_4 TO (d_3T - d_3)/T_[i].
			SET Delta_theta TO Delta_theta + (h_T[i-1]*T_[i]/r_T[i-1]^2+(f_theta[i]*c0(i,T_[i])+fdot_theta[i]*c_(1,i,T_[i])+fdotdot_theta[i]*c_(2,i,T_[i]))/rbar_[i]-d_3*T_[i]^2-d_4*T_[i]^3/3).
		}
	}
}
FUNCTION GuideMajor{
	SET Delta_energy[0] TO Delta_energy[1].
	SET Delta_energy[1] TO Delta_energy[2].
	
	SET energyTime[0] TO energyTime[1].
	SET energyTime[1] TO energyTime[2].
	
	SET energy TO v_^2-2*mu/r_.
	SET Delta_energy[2] TO energy_T[nStage]-energy.
	SET energyTime[2] TO t.
	IF T_[nStage]>10 AND j < nStage {
		SET cutoffEnable TO FALSE.
		SET aa TO b0(j,T_[j]) + b0(j+1,T_[j+1]).
		SET bb TO b_(1,j,T_[j]) + b_(1,j+1,T_[j+1]) + b0(j+1,T_[j+1])*T_[j].
		SET cc TO c0(j,T_[j]) + c0(j+1,T_[j+1]) + b0(j,T_[j])*T_[j+1].
		SET dd TO c_(1,j,T_[j]) + b_(1,j,T_[j])*T_[j+1] + c0(j+1,T_[j+1])*T_[j] + c_(1,j+1,T_[j+1]).
		SET k_b TO rdot_T[j+1] - rdot - b0(j+1,T_[j+1])*Delta_A[j]-b_(1,j+1,T_[j+1])*Delta_B[j].
		SET k_c TO r_T[j+1] - r_ - rdot*(T_[j]+T_[j+1])-c0(j+1,T_[j+1])*Delta_A[j] - c_(1,j+1,T_[j+1])*Delta_B[j].
		
		SET B_[j] TO (k_c*aa-cc*k_b)/(dd*aa-cc*bb).
		SET A_[j] TO k_b/aa-bb*B_[j]/aa.
	}
	ELSE IF T_[nStage] > 10 AND j = nStage{
		SET cutoffEnable TO FALSE.
		SET aa TO b0(j,T_[j]).
		SET bb TO b_(1,j,T_[j]).
		SET cc TO c0(j,T_[j]).
		SET dd TO c_(1,j,T_[j]).
		SET k_b TO rdot_T[j] - rdot.
		SET k_c TO r_T[j] - r_ - rdot*T_[j].
		
		SET B_[j] TO (k_c*aa-cc*k_b)/(dd*aa-cc*bb). 
		SET A_[j] TO (k_b - bb*B_[j])/aa.
	}
	ELSE {
		SET cutoffEnable TO TRUE.
		LOCAL x0 IS energyTime[0]. LOCAL y0 IS Delta_energy[0].
		LOCAL x1 IS energyTime[1]. LOCAL y1 IS Delta_energy[1].
		LOCAL x2 IS energyTime[2]. LOCAL y2 IS Delta_energy[2].
		LOCAL Delta IS x1*x2*x2+x0*x1*x1+x0*x0*x2 - x1*x1*x2 - x0*x2*x2 - x0*x0*x1.
		LOCAL DeltaC IS y0*x1*x2*x2 +x0*x1*x1*y2 +x0*x0*y1*x2 - y0*x1*x1*x2 - x0*y1*x2*x2 - x0*x0*x1*y2.
		LOCAL DeltaB IS y1*x2*x2 +y0*x1*x1   + x0*x0*y2    -   x1*x1*y2 -   y0*x2*x2 -x0*x0*y1.
		LOCAL DeltaA IS x1*y2 +x0*y1       +  y0*x2 - y1*x2       - x0*y2       - y0*x1.
		LOCAL CCC IS DeltaC/Delta.
		LOCAL BBB IS DeltaB/Delta.
		LOCAL AAA IS DeltaA/Delta.
		LOCAL Q IS 0.
		IF BBB > 0{
			SET Q TO - (BBB+1*sqrt(BBB^2-4*AAA*CCC))/2.
		}
		IF BBB < 0{
			SET Q TO - (BBB-1*sqrt(BBB^2-4*AAA*CCC))/2.
		}
		LOCAL t1 IS Q/AAA.
		LOCAL t2 IS CCC/Q.
		IF t1>t and t1<t2 or t2<t{
			SET t_cutoff TO t1.
		}
		IF t1<t and t1>t2 or t2>t{
			SET t_cutoff TO t2.
		}
	}
}
FUNCTION GuideMinor{
	IF acc_[j] > 0{
		SET C_[0] TO (mu/r_^2 - vtheta^2/r_)/acc_[j].
	}
	SET fhatdot_rhat TO A_[j] - B_[j]*(t-tlast)+ C_[0].
	SET fhatdot_hhat TO 0.
	//SET fhatdot_thetahat TO sqrt(1-fhatdot_rhat*fhatdot_rhat-fhatdot_hhat*fhatdot_hhat).
	//SET fhat TO (vdot(rhat,fhatdot_rhat)+vdot(hhat,fhatdot_hhat)) + vdot(thetahat,fhatdot_thetahat).
}
FUNCTION MinorCycle{
	GuideMinor().
}
FUNCTION cutoff{
	//IF t>555 { return true.}
	IF cutoffEnable {return(t>t_cutoff).}
	return false.
}
FUNCTION getPitch {
	DECLARE PARAMETER x.
	DECLARE PARAMETER y.
	DECLARE PARAMETER z.
	
	LOCAL n IS x:LENGTH.
	LOCAL k IS 0.
	
	//	Finds where on the pitch program time table are we now.
	FROM { LOCAL i IS 0. } UNTIL i = n STEP { SET i TO i+1. } DO {
		IF Z < x[i] { SET k TO i. BREAK. }.
	}
	IF k = 0 RETURN y[n-1].
	
	//	Linear interpolation from two values we're between.
	LOCAL m IS (y[k]-y[k-1])/(x[k]-x[k-1]).
	LOCAL b IS y[k]-m*x[k].
	
	RETURN m*t+b.
}
function thrott
{
	declare parameter Dyn_Pressure.
	declare parameter TWR.
	LOCAL MaxQ IS Dyn_Pressure/100.
	LOCAL CurrentQ IS SHIP:Q.
	LOCAL maxAcc IS TWR*g0.
	IF SHIP:MAXTHRUST = 0 {return 1.}
	LOCAL v_maxAcc IS SHIP:MAXTHRUST/SHIP:MASS.
	LOCAL v_minAcc IS 0.4*SHIP:MAXTHRUST/SHIP:MASS.
	LOCAL OUT IS CurrentQ/MaxQ.
	LOCAL desiredQ IS max(0.01,min(1,(1-15*(OUT - 1)))).
	LOCAL desiredTWR IS MAX(0.01,min(1,(maxAcc - v_minAcc)/(v_maxAcc - v_minAcc))).
	LOCAL desired IS max(0.01,min(desiredQ,desiredTWR)).
	return desired.
}
function inst_az {
	parameter
		inc. // target inclination
	
	// find orbital velocity for a circular orbit at the current altitude.
	if inc < ship:latitude and inc > -ship:latitude{return 90.}
	local V_orb is sqrt( body:mu / ( ship:altitude + body:radius)).
	
	// project desired orbit onto surface heading
	local az_orb is arcsin ( cos(inc) / cos(ship:latitude)).
	if (inc < 0) {
		set az_orb to 180 - az_orb.
	}
	
	// create desired orbit velocity vector
	local V_star is heading(az_orb, 0)*v(0, 0, V_orb).

	// find horizontal component of current orbital velocity vector
	local V_ship_h is ship:velocity:orbit - vdot(ship:velocity:orbit, up:vector)*up:vector.
	
	// calculate difference between desired orbital vector and current (this is the direction we go)
	local V_corr is V_star - V_ship_h.
	
	// project the velocity correction vector onto north and east directions
	local vel_n is vdot(V_corr, ship:north:vector).
	local vel_e is vdot(V_corr, heading(90,0):vector).
	
	// calculate compass heading
	local az_corr is arctan2(vel_e, vel_n).
	return az_corr.
}
FUNCTION update_cons{
	IF acc_[j] > 1 {SET tau_[j] TO v_e[j]/acc_[j].}
	SET Delta_t TO t - tlast.
	SET f_r[j] TO f_r[j] + fdot_r[j]*Delta_t.
	SET A_[j] TO A_[j] + B_[j]*Delta_t.
	
	
}
