clear 
clc

%read in image
I = imread("test.jpg");
I = rgb2gray(I);
I = double(I);

%calculate filters
g1 = gaussian_filter(I,5);
g2 = gaussian_filter(I,20);
g3 = gaussian_filter(I,240);

%apply functions
r = msr(I,g1,g2,g3);
g = gain_offset(r);
g = lum(I,g);

%show images
subplot(3,1,1),imshow(I,[])
title("original image")
subplot(3,1,2),imshow(r,[])
title("MSR image")
subplot(3,1,3),imshow(g,[])
title("luminence preservation and gain offset")


%function for 2D Gaussian filter
function [g,filt] = gaussian_filter(f,sigma)
    [M, N] = size(f);
    [u, v] = meshgrid(1:N, 1:M);
    filt = exp(-1*(((u-N/2).^2+(v-M/2).^2)/(2*pi*sigma^2)));
    g = conv2(f,filt,'same');
end

%function for multi scale retinex
function r = msr(f,g1,g2,g3)
    r1 = log(double(f))-log(double(g1));
    r2 = log(double(f))-log(double(g2));
    r3 = log(double(f))-log(double(g3));
    r = (r1+r2+r3)/3;
    
end

%function for gain-offset
function g = gain_offset(f)
    G = 150;
    b = 0.6;
    g = G*(f+b);
    
end

%function for preserving luminence
function g = lum(f, r)
    f_scaled = mat2gray(f);
    r_scaled = mat2gray(r);
    g = (abs(r_scaled-f_scaled)+r_scaled-f_scaled)/2 + f_scaled;
    
end

function g = intScaling4e(f, mode, type)

    if nargin == 1
        type = 'floating';
        if isa(f, 'uint8')
            mode = 'default';
        elseif min(f(:)) < 0 || max(f(:)) > 1
            mode = 'full';
            type = 'floating';
        else
            mode = 'default';
        end
    elseif nargin == 2
            type = 'floating';
    end
    
    NC = size(f,3);
    maxall = max(f(:));

    for I = 1:NC
        g(:,:,I) = intScaleGrayImage(f(:,:,I), maxall, mode, type);
    end
end

function g = intScaleGrayImage(f, maxall, mode, type)
    g = double(f);
    switch mode
        case 'default'
            if maxall > 255
                g = g/maxall;
            elseif isa(f, 'uint8')
                g = double(g)/255;
            else
                g = f;
            end
        case 'full'
            g = g - min(g(:));
            g = g / max(g(:));
    end

    if isequal(type, 'integer')
        if(isToolboxAvailable('Image Processing Toolbox'))
            g = im2uint8(g);
        else
            g = uint8(floor(g*255));
        end
    end
end


