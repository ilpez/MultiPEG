PARAMETER Tgt_Obt.
PARAMETER Tgt_Inc.
PARAMETER Stg.
RUN ONCE guidance_lib.ks.
RUN ONCE Solver.ks.
GLOBAL Linc IS Tgt_Inc.
GLOBAL Lpitch IS 90.
GLOBAL Lazimuth IS 0.
GLOBAL Lsteer IS HEADING(Lazimuth,Lpitch).
GLOBAL Lthrottle IS 1.
SAS OFF.
LOCK STEERING TO Lsteer.
LOCK THROTTLE TO Lthrottle.
GLOBAL SpeedT IS LIST(0,55,100,1000).
GLOBAL TimeT IS LIST(0,0,0,0).
GLOBAL PitchT IS LIST(0,0,0,45.000).
LOCAL StageCount IS STAGE:NUMBER.
UNTIL t = 0 {
	CLEARSCREEN.
	
	SET t TO t + 1.
	IF t = -3 {STAGE.}
	PRINT "TIME " + t + "s".
	WAIT 1.
}
STAGE.
SET t0 TO TIME:SECONDS.

UNTIL FALSE {
	SET t TO TIME:SECONDS - t0.
	IF SHIP:AIRSPEED >= 55{
		SET PitchT[2] TO SHIP:VELOCITY:SURFACE:MAG/t.
		SET PitchT[3] TO PitchT[2].
		SET TimeT[1] TO t.
		SET TimeT[2] TO TimeT[1]+PitchT[2].
		SET TimeT[3] TO TimeT[2]+3.
		BREAK.
	}
	//SET Lazimuth TO inst_az(Linc).
	//SET Lsteer TO HEADING(Lazimuth,Lpitch).
	PRINT "TIME  "+ t + "s".
	WAIT 0.
}.

UNTIL ARCTAN2(SHIP:GROUNDSPEED,SHIP:VERTICALSPEED) > PitchT[3]{
	SET t TO TIME:SECONDS - t0.
	SET Lpitch TO 90-getPitch(TimeT,PitchT,t).
	SET Lazimuth TO inst_az(Linc).
	SET Lthrottle TO thrott(28,5).
	SET Lsteer TO HEADING(Lazimuth,Lpitch).
	PRINT "TIME " + t + "s".
	WAIT 0.
}

UNTIL FALSE {
	SET t TO TIME:SECONDS - t0.
	SET Lpitch TO 90-ARCTAN2(SHIP:GROUNDSPEED,SHIP:VERTICALSPEED).
	SET Lazimuth TO inst_az(Linc).
	SET Lsteer TO HEADING(Lazimuth,Lpitch).
	SET Lthrottle TO thrott(28,5).
	//IF SHIP:VELOCITY:ORBIT:MAG > 4500{E_cutoff().}
	IF STAGIN(){
		SET t1 TO t + 5.
		BREAK.
	}
	PRINT "TIME " + t + "s".
	WAIT 0.
}
FUNCTION STAGIN{
	if stage:number > 0 {
		if maxthrust = 0 {
			stage.
			return true.
		}
		SET numOut to 0.
		LIST ENGINES IN engines. 
		FOR eng IN engines 
		{
			IF eng:FLAMEOUT 
			{
				SET numOut TO numOut + 1.
			}
		}
		if numOut > 0 { 
			stage.
			return true.			
		}.
	}
	return false.
}
UNTIL t > t1{
	SET t TO TIME:SECONDS - t0.
	STAGIN().
	LOCAL e_thr IS 0.
	LOCAL e_isp IS 0.
	LIST ENGINES IN ENGINES.
	FOR e IN engines {
		//	sum specific impulses of all active engines:
		//	Isp_sum = sum(Isp_i*thrust_i) / sum(thrust_i)
		//	note that the following does not do division, only two sums
		IF e:IGNITION {
			SET e_thr TO e_thr + e:THRUST.
			SET e_isp TO e_isp + e:ISP.
		}.
	}.
	SET thrust_ TO e_thr.
	SET Lthrottle TO thrott(28,5).
	RCS ON.
	CLEARSCREEN.
	PRINT "TIME " +t+"s".
	PRINT "Let the engine spool up".
	PRINT "Engine Thrust : " + thrust_ +" kN".
	WAIT 0.
}
GLOBAL Tgt_Alt IS 0.
GLOBAL Tgt_Vy IS 0.
GLOBAL Tgt_Vx IS 0.
SET t TO TIME:SECONDS - t0.
SET Tgt_Alt TO Tgt_Obt*1000 + BODY:RADIUS.
SET Tgt_Vy TO 0.
SET Tgt_Vx TO SQRT(mu/Tgt_Alt).
peg_init(Stg,Tgt_Alt,Tgt_Vy,Tgt_Vx,t).
MajorCycle().
MinorCycle().
GLOBAL tMajor IS t.
UNTIL FALSE{
	SET t TO TIME:SECONDS - t0.
	LOCAL lastMajor IS t - tMajor.
	LOCAL MajorFreq IS 0.8.
	Navigate().
	IF thrust_/mass_ > 1{
		IF lastMajor >= MajorFreq{
			MajorCycle().
			SET tMajor TO t.
			SET lastMajor TO 0.
			//STAGIN().
		}
	MinorCycle().
	STAGIN().
	}
	SET SinPitch TO fhatdot_rhat.
	SET Lpitch TO MAX(-1,MIN(1,SinPitch)).
	SET Lpitch TO ARCSIN(Lpitch).
	SET Lazimuth TO inst_az(Linc).
	SET Lthrottle TO thrott(28,5).
	SET Lsteer TO HEADING(Lazimuth,Lpitch).
	CLEARSCREEN.
	PRINT "TIME " + ROUND(t,2) + "s".
	PRINT "LAST MC " + ROUND(tlast,2) +"s".
	PRINT "fdot_r " + fdot_r[j].
	PRINT "f_r " + f_r[j].
	PRINT "f_rT " + f_rT[j].
	PRINT "f_theta " + f_theta[Stg].
	PRINT "fdot_theta " + fdot_theta[Stg].
	PRINT "fdotdot_theta " + fdotdot_theta[Stg].
	PRINT "D " + D.
	PRINT "N " + N.
	PRINT "Delta V :"+Delta_v.
	PRINT "CUTOFF IN " + T_[Stg] + "s".
	PRINT "CURRENT STAGE : STAGE " + j.
	PRINT "Delta A : " + Delta_A[j].
	PRINT "Delta B : " + Delta_B[j].
	PRINT "A2 : " + A_[stg].
	PRINT "B2 : " + B_[stg].
	PRINT "A1 : " + A_[j].
	PRINT "B1 : " + B_[j].
	PRINT "C : " + C_[0].
	PRINT "C_T: " + C_T[Stg].
	PRINT "Sin pitch : "+ SinPitch.
	PRINT "Pitch : " + Lpitch.
	LOCAL CO IS cutoff().
	IF CO {
		E_cutoff().
		BREAK.
	}
	//STAGIN().
	WAIT 0.
}
FUNCTION E_cutoff{
	LIST ENGINES IN Eng.
	FOR E IN Eng{
		IF E:IGNITION AND E:ALLOWSHUTDOWN{
			E:SHUTDOWN.
		}
	}
}