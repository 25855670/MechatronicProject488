function n = undistortVideoFile(f, K)
%UNDISTORTVIDEOFILE  Re-encode video file f with lens distortion removed
%   using camera intrinsics K. Done after the run because undistorting
%   every frame inside the control loop would slow it down. Returns the
%   number of frames written.
vr  = VideoReader(f);
tmp = [tempname '.mp4'];
vo  = VideoWriter(tmp, 'MPEG-4');
vo.FrameRate = vr.FrameRate;
open(vo);
guard = onCleanup(@() close(vo));

n = 0;
while hasFrame(vr)
    writeVideo(vo, undistortImage(readFrame(vr), K));
    n = n + 1;
end

clear guard vr;                 % close both files before replacing the original
delete(f);
movefile(tmp, f);
end
