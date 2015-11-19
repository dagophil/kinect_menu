#include <iostream>
#include<SFML/Graphics.hpp>
#include "kinect.hxx"

int main()
{
    size_t WIDTH = 800;
    size_t HEIGHT = 600;

    // Fenster anlegen mit 800x600 Aufloesung.
    sf::RenderWindow window(sf::VideoMode(WIDTH,HEIGHT),"Kinect Menu", sf::Style::Close);

    // Sprite fuer Tiefendaten anlegen.
    Kinect k;
    sf::Texture depth_texture;
    if (!depth_texture.create(k.XRes(), k.YRes()))
        throw std::runtime_error("Could not create texture.");
    sf::Sprite depth_sprite(depth_texture);

    while (window.isOpen()){
        sf::Event event;
        while (window.pollEvent(event)){
            if(event.type == sf::Event::Closed){
                window.close();
            }
        }

        window.clear();

        // Warte auf neue Kinect Daten (FPS wird dadurch auf 30 beschraenkt).
        k.wait_for_update();

        // Zeichne das Tiefenbild der Kinect.
        depth_texture.update(k.depth_rgba_ptr());
        depth_sprite.setScale(WIDTH / (float)k.XRes(), HEIGHT / (float)k.YRes());
        window.draw(depth_sprite);

        window.display();
    }
}
