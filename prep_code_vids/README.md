# Concatenate Videos

In order to do video coding, videos must be concatenated and renamed. In order to do this, you must:

1.  Get videos onto your machine (local or remote)
2.  Run the concatenation script
3.  Change the filename to be standardized
4.  Put the files onto Penn+Box

## Get videos onto your machine

You can take the standard method of downloading videos via the gui. This is not the best.
Alternatively you can create a password on Penn+Box (not your penn password) and then
mount ftp://ftp.box.com/. Once that is mounted, you can use rsync to bring files locally, ex: `target=003 && mkdir -p ~/Downloads/sdata/$target/gopro/ && rsync -ah --info=progress2 "/run/user/1001/gvfs/ftp:host=ftp.box.com/RRL Studies/Flo Prospective Study/trials/$target/gopro/" ~/Downloads/sdata/$target/gopro/` (your paths will differ) (change the target value to the target subject)

You can also use lftp (faster, only need terminal). Refer to main readme

You could also use a tool like filezilla

## Run concatenation script

The concatenation script needs to be given a folder to look for gopro videos in and an output folder. For example: `target=003 && ~/Documents/git/LilFloAssessmentPipeline/prep_code_vids/concatenate_vids.sh -t ~/Downloads/sdata/$target/gopro -o ~/Downloads/sdata_out/$target/gopro`

## Rename files

Your concatenated files should now be in a folder somewhere. Rename them to be: `<ID ###>-<C/I/A>-<interaction ##>`

## Put files on Penn+Box

You can either drag and drop files back to Penn+Box or mount the FTP enpoint and copy them there.
