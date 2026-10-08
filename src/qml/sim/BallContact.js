// The list is newest first. Prefer the latest incoming sample, never the fastest old one.
function incomingVelocity(samples, px, pz, fx, fz, maxNormalDelta) {
    for (var k = 0; k < samples.length; ++k) {
        var normal = (samples[k].x * 1000 - px) * fx + (samples[k].z * 1000 - pz) * fz;
        if (normal >= 0) continue;
        if (k + 1 < samples.length && maxNormalDelta > 0) {
            var older = (samples[k + 1].x * 1000 - px) * fx + (samples[k + 1].z * 1000 - pz) * fz;
            // A sudden drop in incoming speed spans the collision. Use the preceding interval.
            if (older < 0 && normal - older > maxNormalDelta) continue;
        }
        return samples[k];
    }
    return samples[0];
}

// Allow deferred launch/reset to finish before recovering a ball stopped in the mouth.
// A still-outgoing ball remains protected even when the guard time has elapsed.
function canRecover(ageMs, relativeNormalSpeed, pendingLaunch) {
    return !pendingLaunch && ageMs >= 150 && relativeNormalSpeed <= 20;
}
