/* 
 * File:   ol-vibe.h
 * Author: pullin
 *
 * Created on January 20, 2014, 1:59 PM
 */

#ifndef OL_VIBE_H
#define	OL_VIBE_H

void olVibeSetup();

void olVibeStart(void);
void olVibeStop(void);
void olVibeSetFrequency(unsigned int channel, unsigned int incr);
void olVibeSetAmplitude(unsigned int channel, unsigned int amp);
void olVibeSetAmplitudeFloat(unsigned int channel, float famp);
void olVibeSetPhase(unsigned int channel, short phase); //short is surrogate for _Q15

#endif	/* OL_VIBE_H */

