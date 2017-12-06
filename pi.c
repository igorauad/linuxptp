/**
 * @file pi.c
 * @brief Implements a Proportional Integral clock servo.
 * @note Copyright (C) 2011 Richard Cochran <richardcochran@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <stdlib.h>
#include <math.h>

#include "config.h"
#include "pi.h"
#include "print.h"
#include "servo_private.h"

#define HWTS_KP_SCALE 0.7
#define HWTS_KI_SCALE 0.3
#define SWTS_KP_SCALE 0.1
#define SWTS_KI_SCALE 0.001

#define MAX_KP_NORM_MAX 1.0
#define MAX_KI_NORM_MAX 2.0

#define FREQ_EST_MARGIN 0.001

struct pi_servo {
	struct servo servo;
	int64_t offset[2];
	uint64_t local[2];
	double drift;
	double kp;
	double ki;
	double last_freq;
	int count;
	/* configuration: */
	double configured_pi_kp;
	double configured_pi_ki;
	double configured_pi_kp_scale;
	double configured_pi_kp_exponent;
	double configured_pi_kp_norm_max;
	double configured_pi_ki_scale;
	double configured_pi_ki_exponent;
	double configured_pi_ki_norm_max;
};

static void pi_destroy(struct servo *servo)
{
	struct pi_servo *s = container_of(servo, struct pi_servo, servo);
	free(s);
}

static double pi_sample(struct servo *servo,
			int64_t offset,    // IGOR: gets "master_offset"
			uint64_t local_ts, // IGOR: gets ingress time (timestamp t2)
			double weight,
			enum servo_state *state)
{
	struct pi_servo *s = container_of(servo, struct pi_servo, servo);
	// IGOR: Note, in order to always have something to print, when the
	// servo is in the unlocked state, ppb (the drift) actually prints
	// the current frequency estimation.
	double ki_term, ppb = s->last_freq;
	double freq_est_interval, localdiff;

	switch (s->count) {
	case 0:
		s->offset[0] = offset;
		s->local[0] = local_ts;
		*state = SERVO_UNLOCKED;
		s->count = 1;
		break;
	case 1:
		s->offset[1] = offset;
		s->local[1] = local_ts;

		/* Make sure the first sample is older than the second. */
		if (s->local[0] >= s->local[1]) {
			*state = SERVO_UNLOCKED;
			s->count = 0;
			break;
		}

		/* Wait long enough before estimating the frequency offset. */
		localdiff = (s->local[1] - s->local[0]) / 1e9; // IGOR: difference in seconds
		localdiff += localdiff * FREQ_EST_MARGIN; // IGOR: Adds + 0.1% as margin
		freq_est_interval = 0.016 / s->ki;
		if (freq_est_interval > 1000.0) {
			freq_est_interval = 1000.0;
		}
		if (localdiff < freq_est_interval) {
			*state = SERVO_UNLOCKED;
			break;
		}

		/* Adjust drift by the measured frequency offset. */
		s->drift += (1e9 - s->drift) * (s->offset[1] - s->offset[0]) /
						(s->local[1] - s->local[0]);
		/*
		 * IGOR:
		 * s->drift stores the drift in ppb, which is essentially a cumsum
		 * of the instantaneous freq. offsets. Details in
		 * github.com/richardcochran/linuxptp-as/commit/2ca067dcace97e1b5c28f1da3755b9d33f02bf0b
		 * A discussion about this implementation can be found in:
		 * linuxptp-devel.narkive.com/jcrIBJE1/patch-fix-drift-calculation-in-pi-servo-with-large-values
		 * IGOR: consider also that the "s->drift" term in (1e9 - s->drift)
		 * has very small contribution, since it multiples something the
		 * fractional frequency offset (which should be very small). In
		 * contrast, the "1e9" converts to ppb, so it is significant.
		 */

		// IGOR: Limit the drift
		if (s->drift < -servo->max_frequency)
			s->drift = -servo->max_frequency;
		else if (s->drift > servo->max_frequency)
			s->drift = servo->max_frequency;

		/* IGOR
		 * If the current iteration is the first update ever made (i.e.
		 * the first time that the servo leaves the SERVO_UNLOCKED state),
		 * then check to see if the "offset from master" is not beyond
		 * acceptable limits, that is, the limits within which a frequency
		 * adjustment can correct.
		 * 	If the offset is in fact above these limits, then go to the JUMP
		 * state, where the clock will be stepped. Nonetheless, note that the
		 * drift is still corrected, since the ppb value is returned. The
		 * difference is that, besides the freq correction itself, the clock
		 * is also stepped.
		 */
		if ((servo->first_update &&
		     servo->first_step_threshold &&
		     servo->first_step_threshold < llabs(offset)) ||
		    (servo->step_threshold &&
		     servo->step_threshold < llabs(offset)))
			*state = SERVO_JUMP;
		else
			*state = SERVO_LOCKED;

		ppb = s->drift;
		s->count = 2;
		break;
	case 2:
		/*
		 * reset the clock servo when offset is greater than the max
		 * offset value. Note that the clock jump will be performed in
		 * step 1, so it is not necessary to have clock jump
		 * immediately. This allows re-calculating drift as in initial
		 * clock startup.
		 */
		/* IGOR:
		 *	To put differently, if the current "master offset" is greater
		 * than a threshold, no frequency adjustment is attempted, from the
		 * rationale that it is too big to correct via frequency. The servo,
		 * then, goes back to the UNLOCKED state to reinitiate the state
		 * machie. In the outside world where "pi_sample" is called, the
		 * clock jump (step) will occurr right after coming through this case
		 * 2, since there the state is still SERVO_JUMP. Then, in the next
		 * iteration, the process begins back from the UNLOCKED state.
		 */
		if (servo->step_threshold &&
		    servo->step_threshold < llabs(offset)) {
			*state = SERVO_UNLOCKED;
			s->count = 0;
			break;
		}
		/*
		* IGOR:
		* Note the difference from Hamza Abubakari's paper. There, a
		* cascade skew estimator precedes the PI controller, which implies
		* that the input to the PI controller is the skew estimation. Here,
		* the input to the PI controller is directly the "offset from master"
		* as described in Cochran's paper.
		*/
		ki_term = s->ki * offset * weight;
		ppb = s->kp * offset * weight + s->drift + ki_term;
		// IGOR: Limit the ppb correction (saturation filter)
		if (ppb < -servo->max_frequency) {
			ppb = -servo->max_frequency;
		} else if (ppb > servo->max_frequency) {
			ppb = servo->max_frequency;
		} else {
			s->drift += ki_term;
		}
		// IGOR: Keep locked (s->count maintained in 2)
		*state = SERVO_LOCKED;
		break;
	}

	s->last_freq = ppb;
	return ppb;
}

static void pi_sync_interval(struct servo *servo, double interval)
{
	//IGOR: interval comes from "logSyncInterval" in the cfg

	struct pi_servo *s = container_of(servo, struct pi_servo, servo);

	// IGOR: configured_pi_kp_scale default is 0.0
	s->kp = s->configured_pi_kp_scale * pow(interval, s->configured_pi_kp_exponent);
	if (s->kp > s->configured_pi_kp_norm_max / interval)
		s->kp = s->configured_pi_kp_norm_max / interval;

	s->ki = s->configured_pi_ki_scale * pow(interval, s->configured_pi_ki_exponent);
	if (s->ki > s->configured_pi_ki_norm_max / interval)
		s->ki = s->configured_pi_ki_norm_max / interval;

	pr_debug("PI servo: sync interval %.3f kp %.3f ki %.6f",
		 interval, s->kp, s->ki);
}

static void pi_reset(struct servo *servo)
{
	struct pi_servo *s = container_of(servo, struct pi_servo, servo);

	s->count = 0;
}

struct servo *pi_servo_create(struct config *cfg, int fadj, int sw_ts)
{
	struct pi_servo *s;

	s = calloc(1, sizeof(*s));
	if (!s)
		return NULL;

	s->servo.destroy = pi_destroy;
	s->servo.sample  = pi_sample;
	s->servo.sync_interval = pi_sync_interval;
	s->servo.reset   = pi_reset;
	s->drift         = fadj;
	s->last_freq     = fadj;
	s->kp            = 0.0;
	s->ki            = 0.0;
	s->configured_pi_kp = config_get_double(cfg, NULL, "pi_proportional_const");
	s->configured_pi_ki = config_get_double(cfg, NULL, "pi_integral_const");
	s->configured_pi_kp_scale = config_get_double(cfg, NULL, "pi_proportional_scale");
	s->configured_pi_kp_exponent =
		config_get_double(cfg, NULL, "pi_proportional_exponent");
	s->configured_pi_kp_norm_max =
		config_get_double(cfg, NULL, "pi_proportional_norm_max");
	s->configured_pi_ki_scale =
		config_get_double(cfg, NULL, "pi_integral_scale");
	s->configured_pi_ki_exponent =
		config_get_double(cfg, NULL, "pi_integral_exponent");
	s->configured_pi_ki_norm_max =
		config_get_double(cfg, NULL, "pi_integral_norm_max");

	if (s->configured_pi_kp && s->configured_pi_ki) {
		/* Use the constants as configured by the user without
		   adjusting for sync interval unless they make the servo
		   unstable. */
		s->configured_pi_kp_scale = s->configured_pi_kp;
		s->configured_pi_ki_scale = s->configured_pi_ki;
		s->configured_pi_kp_exponent = 0.0;
		s->configured_pi_ki_exponent = 0.0;
		s->configured_pi_kp_norm_max = MAX_KP_NORM_MAX;
		s->configured_pi_ki_norm_max = MAX_KI_NORM_MAX;
	} else if (!s->configured_pi_kp_scale || !s->configured_pi_ki_scale) {
		if (sw_ts) {
			s->configured_pi_kp_scale = SWTS_KP_SCALE;
			s->configured_pi_ki_scale = SWTS_KI_SCALE;
		} else {
			s->configured_pi_kp_scale = HWTS_KP_SCALE;
			s->configured_pi_ki_scale = HWTS_KI_SCALE;
		}
	}

	return &s->servo;
}
